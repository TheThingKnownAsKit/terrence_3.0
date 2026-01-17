
3. Fill out all the other ROS2 Control stuff like the controller yaml files
4. Create launch files to test this empty hardware component
5. Create an arduino comms toolkit for the teensy
6. Create basic teensy firmware that just sends back a message when it receives a message
7. Fill out the hardware component method stubs to use this arduino comms with fluff data
8. Test the arduino comms using this basic firmware + a teensy
9. Make the hardware component accurately reflect desired logic (full logic implementation)
10. Make sure the teensy firmware can be tested with this full logic but cant send real data back so fluff the data
11. Final tests

Bonus: get sim to work again