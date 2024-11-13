- ros2 action commandline tools:
  ```shell
  ros2 action list

  ros2 action list -t # list actions with their corresponding interface types

  ros2 action info <action_name>

  ros2 topic list --include-hidden-topics

  ros2 service list --include-hidden-topics

  ros2 action send_goal <action_name> <action_type> "{}" --feedback

  ros2 action send_goal /count_until my_practice3_interface/action/CountUntil "{target_number: 4, period_btw_count: 1.5}"
  ```