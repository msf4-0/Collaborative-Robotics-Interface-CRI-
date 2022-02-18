Blockly.Python['teachpoints_storej'] = function(block) {
  var value_j1 = Blockly.Python.valueToCode(block, 'J1', Blockly.Python.ORDER_ATOMIC);
  var value_j2 = Blockly.Python.valueToCode(block, 'J2', Blockly.Python.ORDER_ATOMIC);
  var value_j3 = Blockly.Python.valueToCode(block, 'J3', Blockly.Python.ORDER_ATOMIC);
  var value_j4 = Blockly.Python.valueToCode(block, 'J4', Blockly.Python.ORDER_ATOMIC);
  var value_j5 = Blockly.Python.valueToCode(block, 'J5', Blockly.Python.ORDER_ATOMIC);
  var value_j6 = Blockly.Python.valueToCode(block, 'J6', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code =  value_j1 + ',' + value_j2 + ',' + value_j3 + ',' + value_j4 + ',' + value_j5 + ',' + value_j6;
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_MEMBER];
};

Blockly.Python['movej'] = function(block) {
  var value_movej = Blockly.Python.valueToCode(block, 'MoveJ', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = '    client_feedback.JointMovJ' + value_movej + '\n    time.sleep(5)\n';
  return code;
};

Blockly.Python['teachpoints_storel'] = function(block) {
  var value_x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_ATOMIC);
  var value_y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_ATOMIC);
  var value_z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_ATOMIC);
  var value_rx = Blockly.Python.valueToCode(block, 'RX', Blockly.Python.ORDER_ATOMIC);
  var value_ry = Blockly.Python.valueToCode(block, 'RY', Blockly.Python.ORDER_ATOMIC);
  var value_rz = Blockly.Python.valueToCode(block, 'RZ', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code =  value_x + ',' + value_y + ',' + value_z + ',' + value_rx + ',' + value_ry + ',' + value_rz ;
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_MEMBER];
};

Blockly.Python['movel'] = function(block) {
  var value_movel = Blockly.Python.valueToCode(block, 'MoveL', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = '    client_feedback.MovL' + value_movel + '\n    time.sleep(5)\n';
  return code;
};

Blockly.Python['accelj'] = function(block) {
  var value_accelj = Blockly.Python.valueToCode(block, 'AccelJ', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = '    client_dashboard.AccJ' + value_accelj + '\n';;
  return code;
};

Blockly.Python['accell'] = function(block) {
  var value_accell = Blockly.Python.valueToCode(block, 'AccelL', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = '    client_dashboard.AccL' + value_accell + '\n';
  return code;
};

Blockly.Python['speedj'] = function(block) {
  var value_speedj = Blockly.Python.valueToCode(block, 'SpeedJ', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = '    client_dashboard.SpeedJ' +  value_speedj + '\n';
  return code;
};

Blockly.Python['speedl'] = function(block) {
  var value_speedl = Blockly.Python.valueToCode(block, 'SpeedL', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = '    client_dashboard.SpeedL' +  value_speedl + '\n';
  return code;
};

Blockly.Python['arc'] = function(block) {
  var value_arcm = Blockly.Python.valueToCode(block, 'ArcM', Blockly.Python.ORDER_FUNCTION_CALL);
  var value_arce = Blockly.Python.valueToCode(block, 'ArcE', Blockly.Python.ORDER_FUNCTION_CALL);
  // TODO: Assemble Python into code variable.
  var code = '    client_feedback.Arc(' + value_arcm + ',' + value_arce + ')\n    time.sleep(5)\n';
  return code;
};

Blockly.Python['start'] = function(block) {
  // TODO: Assemble Python into code variable.
  var code = 'from dobot_api import dobot_api_dashboard, dobot_api_feedback, MyType\nfrom threading import Thread\nfrom multiprocessing import Process\nimport numpy as np\nimport time\ndef main(client_dashboard, client_feedback):\n    # Remove alarm\n    client_dashboard.ClearError()\n    time.sleep(0.5)\n    # Description The upper function was enabled successfully\n    client_dashboard.EnableRobot()\n    time.sleep(0.5)\n';
  return code;
};

Blockly.Python['end'] = function(block) {
  // TODO: Assemble Python into code variable.
  var code = '# The feedback information about port 30003 is displayed\ndef data_feedback(client_feedback):\n    while True:\n        time.sleep(0.05)\n        all = client_feedback.socket_feedback.recv(10240)\n        data = all[0:1440]\n        a = np.frombuffer(data, dtype=MyType)\n        a = np.frombuffer(data, dtype=MyType)\n        if hex((a[\'test_value\'][0])) == \'0x123456789abcdef\':\n            print(\'robot_mode\', a[\'robot_mode\'])\n            print(\'tool_vector_actual\', np.around(a[\'tool_vector_actual\'], decimals=4))\n            print(\'q_actual\', np.around(a[\'q_actual\'], decimals=4))\n\n'+
  '# Enable threads on ports 29999 and 30003\nif __name__ == \'__main__\':\n\tclient_dashboard = dobot_api_dashboard(\'192.168.5.1\', 29999)\n\tclient_feedback = dobot_api_feedback(\'192.168.5.1\', 30003)\n\tp1 = Thread(target=main, args=(client_dashboard, client_feedback))\n\tp1.start()\n\tp2 = Thread(target=data_feedback, args=(client_feedback, ))\n\tp2.daemon =True\n\tp2.start()\n\tp1.join()\n\tclient_dashboard.close()\n\tclient_feedback.close()\n';
  return code;
};

Blockly.Python['num_block'] = function(block) {
  var text_num = block.getFieldValue('NUM');
  // TODO: Assemble JavaScript into code variable.
  var code = text_num;
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};