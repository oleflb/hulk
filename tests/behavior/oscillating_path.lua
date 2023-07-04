function spawn_robot(number)
  table.insert(state.robots, create_robot(number))
end

spawn_robot(2)

state.filtered_game_state = {
  Playing = {
    ball_is_free = true
  }
}

function on_cycle()
  if state.cycle_count == 100 then 
    state.ball = {
      position = { 1.0, 0.0 },
      velocity = { 0.0, 0.0 },
    }
    set_robot_pose(2, { -1.0, 0 }, 0);

  end

  if state.cycle_count == 5000 then
    state.finished = true
  end
end
