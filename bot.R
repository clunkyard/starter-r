# Narrow the scan around a contact and lead shots from recent sightings.
# Clunkyard guide: https://clunkyard.com/sdk/r
library(clunkyard)

SEARCH_TURN <- 0.33 # enough to turn against the brake in the force rulesets
SEARCH_ARC <- 40    # degrees: sweeps the circle in half a second
HOLD_WIDTH <- 60    # units across at the target: it stays in the beam from tick to tick
MIN_ARC <- 6        # degrees: the narrowest the beam holds
WINDOW <- 12        # ticks of sightings the aim averages
SIGHTINGS <- 8      # narrow sightings in the window before a shot
LOOK_EVERY <- 40    # ticks between looks around the target, for a nearer robot
REACQUIRE <- 10     # ticks to look again where a lost target was

# A turn or pitch-turn control towards an angle, full at 45 degrees off.
steer <- function(towards, current) angles$clamp(angles$angle_diff(towards, current) / 45)

free <- FALSE # flying free, robots may be above or below
dodges <- 0
dodge_end <- 0
slot <- 0

on_start <- function(bot) free <<- bot$match$is_3d && bot$arena$gravity$z == 0

# A straight line through the narrowest of the last WINDOW ticks' sightings:
# where the robot is now, how fast it moves, and how many sightings it rests
# on. Each sighting is off by its bearing_error; the line averages that out.
fit <- function(bot, slot, tick) {
  recent <- Filter(function(s) s$tick > tick - WINDOW, bot$tracker$history(slot))
  if (length(recent) < 2) return(NULL)
  best <- min(vapply(recent, function(s) s$bearing_error, numeric(1)))
  seen <- Filter(function(s) s$bearing_error <= 1.5 * best, recent)
  n <- length(seen)
  if (n < 2) return(NULL)
  ticks <- vapply(seen, function(s) s$tick, numeric(1))
  t0 <- mean(ticks)
  position <- list()
  velocity <- list()
  for (axis in c("x", "y", "z")) {
    p <- vapply(seen, function(s) s$position[[axis]], numeric(1))
    slope <- sum((ticks - t0) * (p - mean(p))) / sum((ticks - t0)^2)
    position[[axis]] <- mean(p) + slope * (tick - t0)
    velocity[[axis]] <- slope * bot$match$tick_rate
  }
  list(position = position, velocity = velocity, count = n)
}

on_turn <- function(bot) {
  me <- bot$me
  position <- me$position
  tick <- bot$clock$tick
  bot$drive$forces(brake = 0.2)
  target <- bot$sensors$nearest
  cue <- bot$tracker$cue # the freshest passive contact, for a few seconds
  last_seen <- if (slot != 0) bot$tracker$last_seen(slot) else NULL
  if (!is.null(target)) {
    slot <<- target$slot
    throttle <- if (target$range > 350) 0.7 else if (target$range < 200) -0.4 else 0.2
    bot$drive$set(throttle = throttle, turn = steer(target$bearing, me$heading), pitch_turn = steer(target$elevation, me$pitch))
    # Halve the beam each tick until it is HOLD_WIDTH across at the target:
    # narrow enough for a precise bearing, wide enough to keep it in.
    hold <- max(MIN_ARC, 2 * atan2(HOLD_WIDTH / 2, target$range) * 180 / pi)
    # In a crowd a narrow beam sees one robot. Every LOOK_EVERY ticks,
    # widen it for a tick: a nearer robot it finds becomes the target.
    arc <- if (tick %% LOOK_EVERY == 0) SEARCH_ARC else max(hold, min(SEARCH_ARC, bot$scanner$last$arc / 2))
    bot$scanner$set(heading = target$bearing, arc = arc, pitch = target$elevation)
    # The heaviest shot that still covers the range in about a second,
    # aimed where the target will be, lobbed under gravity.
    power <- bot$gun$power_for_flight_time(target$range, 1)
    # An exact contact: gun$lead is the intercept if the target holds
    # course. A noisy one: lead the fitted line, and fire only once it
    # rests on enough sightings.
    ready <- TRUE
    if (target$bearing_error > 0) {
      line <- fit(bot, target$slot, tick)
      aim <- if (is.null(line)) NULL else bot$gun$lead_point(line$position, line$velocity, power)
      ready <- !is.null(line) && line$count >= SIGHTINGS
    } else {
      aim <- bot$gun$lead(target, power)
    }
    heading <- if (is.null(aim)) target$bearing else aim$heading
    pitch <- if (is.null(aim)) target$elevation else aim$pitch
    bot$gun$aim(heading, pitch)
    now <- bot$gun$aim_now
    on_target <- abs(angles$angle_diff(heading, now$heading)) < 3 && abs(pitch - now$pitch) < 3
    if (ready && on_target && bot$gun$can_fire(power, 10)) bot$gun$fire(power)
    bot$status$set(if (ready) "TRACK" else "NARROW", target$range)
  } else if (!is.null(last_seen) && tick - last_seen <= REACQUIRE) {
    # Lost it: look again where it was last seen, widening the beam, before searching.
    where <- bot$tracker$position(slot)
    bearing <- angles$bearing_to(position, where)
    elevation <- if (free) angles$elevation_to(position, where) else 0
    bot$drive$set(throttle = 0.5, turn = steer(bearing, me$heading), pitch_turn = steer(elevation, me$pitch))
    bot$scanner$set(heading = bearing, arc = min(SEARCH_ARC, bot$scanner$last$arc * 2), pitch = elevation)
    bot$status$set("REACQUIRE")
  } else if (!is.null(cue)) {
    # Head for where it was heard, sweeping an arc that still reaches it
    # across the bearing's error (and, flying free, the elevation's).
    bearing <- angles$bearing_to(position, cue$position)
    distance <- angles$distance(position, cue$position)
    elevation <- if (free) angles$elevation_to(position, cue$position) else 0
    error <- min(cue$bearing_error, 30)
    across <- (tick %% 5 - 2) / 2 * error
    up <- if (free) ((tick %/% 5) %% 3 - 1) * error / 2 else 0
    bot$drive$set(throttle = 0.7, turn = steer(bearing, me$heading), pitch_turn = steer(elevation, me$pitch))
    bot$scanner$set(heading = bearing + across, arc = bot$scanner$arc_for_reach(distance + 100), pitch = elevation + up)
    bot$status$set("CUE", distance)
  } else {
    bot$drive$set(throttle = 0.5, turn = SEARCH_TURN, pitch_turn = steer(0, me$pitch))
    # Sweeps the circle in half a second, flying free at three heights in turn.
    pitch <- if (free) ((tick %/% 10) %% 3 - 1) * 35 else 0
    bot$scanner$set(heading = angles$normalize(bot$scanner$last$heading + 36), arc = SEARCH_ARC, pitch = pitch)
    bot$status$set("SEARCH")
  }
  # A simple dodge: vary it, or predict the shooter.
  heard <- vapply(bot$sensors$cues, function(cue) cue$source == "muzzle", logical(1))
  if (tick >= dodge_end && any(heard)) {
    dodges <<- dodges + 1
    dodge_end <<- tick + 10
  }
  if (tick < dodge_end) {
    # Off the line of fire: left, then right next time (turn is positive left, strafe right).
    left <- dodges %% 2 == 1
    if (bot$match$control == "forces") {
      bot$drive$set(throttle = 0.5)
      bot$drive$forces(strafe = if (left) -1 else 1)
    } else {
      bot$drive$set(throttle = 1, turn = if (left) 1 else -1)
    }
    bot$status$set(if (left) "dodge:left" else "dodge:right", dodges)
  }
  # Low battery: drive gently so the shots can still be paid for.
  if (bot$battery$charge < bot$battery$capacity / 4) bot$drive$set(throttle = bot$drive$get("throttle") * 0.5)
  # The storm hurts inside: near its edge, drive straight away from its centre.
  storm <- bot$nav$storm
  if (!is.null(storm) && storm$margin(position) > -60) {
    away <- angles$bearing_to(storm$center, position)
    bot$drive$set(throttle = 0.8, turn = angles$clamp(angles$angle_diff(away, me$heading) / 30))
  }
  # Sudden death: head back inside the safe zone.
  zone <- bot$nav$zone
  if (!is.null(zone) && zone$margin(position) < 60) {
    climb <- if (zone$shape == "sphere") angles$elevation_to(position, zone$center) else 0
    bot$drive$set(throttle = 0.8,
                  turn = angles$clamp(angles$angle_diff(angles$bearing_to(position, zone$center), me$heading) / 30),
                  pitch_turn = angles$clamp((climb - me$pitch) / 30))
  }
}

run(robot(on_turn = on_turn, on_start = on_start))
