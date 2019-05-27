//----------------------------------------ROS----------------------------------------//

// ------------------------------------//
// ROS for this UI
// -----------------------------------//

// var ui_client = new ROSLIB.Service({
//     ros : ros,
//     name : '/ui_server',
//     serviceType : 'mbot_control/UI_Server'
//   });

var gripper_control_client = new ROSLIB.Service({
    ros : ros,
    name : 'gripper_service',
    serviceType : 'comm_stm32/gripper_cmd'
});


var wait_time = 3000
$("#btn_Catch_all").click(function(){
	$(this).removeClass('active');
    $(this).addClass('disabled');
	var request = new ROSLIB.ServiceRequest
	({
		val : 1,
	});

	gripper_control_client.callService(request, function (res) {
		var is_success = res.success;
    });

    setTimeout(function(){$("#btn_stop_all").click();}, wait_time);  // wait n ms, then run function -> $("#btn_stop_all").click(); 
    console.log('click btn_Catch_all');
	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#btn_stop_all").click(function(){
	$(this).removeClass('active');
    $(this).addClass('disabled');
	var request = new ROSLIB.ServiceRequest
	({
		val : 0,
	});

	gripper_control_client.callService(request, function (res) {
		var is_success = res.success;
    });
    console.log('click btn_stop_all');
	$(this).addClass('active');
	$(this).removeClass('disabled');
});