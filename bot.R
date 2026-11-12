# A starter robot for rules v2, on the Clunkyard R binding (the clunkyard
# package, which the arena provides). It sweeps a medium scan to find a
# robot, or points it where a passive contact was heard, narrows the scan to
# hold it, leads it with shots whose power suits the range, fires only when
# the turret is on target, keeps battery back for driving and keeps out of
# the storm.
#
# The trade-offs (bot$gun, bot$scanner and bot$battery have the numbers):
#   scan     a narrow arc reaches further; a wide one finds robots nearby
#   power    heavy shots hit harder for less battery, but fly slowly
#   battery  shots and driving share it, and it refills slowly
#   turret   it turns at a limited rate; bot$gun$aim_now is where it points now
#   cues     passive contacts (a ping, a scan beam, a shot heard) say which
#            way to look, not where to shoot: they have no id and a noisy
#            bearing, so bot$sensors$nearest leaves them out
# Clunkyard robot API 2.0.0: one core under every language, the same
# answers everywhere. Facts (a call shows its arguments):
# match: mode ruleset control robots is_3d tick_rate max_ticks
#   robot_id(slot)
# clock: tick seconds remaining_ticks
# arena: size gravity terrain_kind ground_height(x y)
# me: health position velocity heading pitch roll angular_velocity
#   attitude grounded altitude applied
# battery: charge capacity regen sustainable_drive compute
#   drive_cost(fraction)
# drive: hover_lift
# gun: aim_now cooldown_ticks spread max_power damage_scale
#   can_fire(power reserve) shot_speed(power) shot_cost(power)
#   reload_ticks(power) shot_damage(power) power_for_speed(speed)
#   power_for_flight_time(range seconds) lead(contact power)
#   lead_point(position velocity power)
# scanner: last proximity_range reach(arc) arc_for_reach(range)
# sensors: contacts nearest cues contact(slot) position_of(contact)
# tracker: cue nearest_bearing_rate velocity(slot) position(slot)
#   last_seen(slot)
# nav: zone zone.margin(point) zone.contains(point margin)
#   zone.distance_from(point) storm storm.margin(point)
#   storm.contains(point margin) storm.distance_from(point) hill
#   hill.margin(point) hill.contains(point margin)
#   hill.distance_from(point) pickups nearest_pickup(kind)
#   field_distance(a b)
# score: me best armour
# angles: normalize(degrees) angle_diff(target current)
#   bearing_to(from to) elevation_to(from to) direction(heading pitch)
#   distance(a b) clamp(value low high)
# raw: number(path) text(path) has(path) flag(path) count(path) line()
# Controls: drive.set(throttle turn pitch_turn)
#   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
#   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
#   tracker.configure(smoothing forget_ticks cue_ticks)
#   status.set(text value) raw.extra(json)
# Every control argument is optional; a second call keeps what the first
# set. Absent facts are your language's none. Names are snake_case here;
# your language cases them its own way. Angles are degrees from the +x
# axis; pitch is positive up.
# Manual: https://clunkyard.com/manual#sdk
library(clunkyard)

SEARCH_TURN <- 0.33 # enough to turn against the brake in the force rulesets

# A turn or pitch-turn control towards an angle, full at 45 degrees off.
steer <- function(towards, current) angles$clamp(angles$angle_diff(towards, current) / 45)

free <- FALSE # flying free, robots may be above or below

on_start <- function(bot) free <<- bot$match$is_3d && bot$arena$gravity$z == 0

on_turn <- function(bot) {
  me <- bot$me
  position <- me$position
  bot$drive$forces(brake = 0.2)
  target <- bot$sensors$nearest
  cue <- bot$tracker$cue # the freshest passive contact, for a few seconds
  if (!is.null(target)) {
    throttle <- if (target$range > 350) 0.7 else if (target$range < 200) -0.4 else 0.2
    bot$drive$set(throttle = throttle, turn = steer(target$bearing, me$heading), pitch_turn = steer(target$elevation, me$pitch))
    bot$scanner$set(heading = target$bearing, arc = 20, pitch = target$elevation) # narrow: holds it, reaching about 735
    # The heaviest shot that still covers the range in about a second,
    # aimed where the target will be, lobbed under gravity.
    power <- bot$gun$power_for_flight_time(target$range, 1)
    aim <- bot$gun$lead(target, power)
    heading <- if (is.null(aim)) target$bearing else aim$heading
    pitch <- if (is.null(aim)) target$elevation else aim$pitch
    bot$gun$aim(heading, pitch)
    now <- bot$gun$aim_now
    on_target <- abs(angles$angle_diff(heading, now$heading)) < 3 && abs(pitch - now$pitch) < 3
    if (on_target && bot$gun$can_fire(power, 10)) bot$gun$fire(power)
    bot$status$set("TRACK", target$range)
  } else if (!is.null(cue)) {
    # Head for where it was heard, sweeping an arc that still reaches it
    # across the bearing's error (and, flying free, the elevation's).
    bearing <- angles$bearing_to(position, cue$position)
    distance <- angles$distance(position, cue$position)
    elevation <- if (free) angles$elevation_to(position, cue$position) else 0
    error <- min(cue$bearing_error, 30)
    tick <- bot$clock$tick
    across <- (tick %% 5 - 2) / 2 * error
    up <- if (free) ((tick %/% 5) %% 3 - 1) * error / 2 else 0
    bot$drive$set(throttle = 0.7, turn = steer(bearing, me$heading), pitch_turn = steer(elevation, me$pitch))
    bot$scanner$set(heading = bearing + across, arc = bot$scanner$arc_for_reach(distance + 100), pitch = elevation + up)
    bot$status$set("CUE", distance)
  } else {
    bot$drive$set(throttle = 0.5, turn = SEARCH_TURN, pitch_turn = steer(0, me$pitch))
    # Sweeps the circle in half a second, flying free at three heights in turn.
    pitch <- if (free) ((bot$clock$tick %/% 10) %% 3 - 1) * 35 else 0
    bot$scanner$set(heading = angles$normalize(bot$scanner$last$heading + 36), arc = 40, pitch = pitch)
    bot$status$set("SEARCH")
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
