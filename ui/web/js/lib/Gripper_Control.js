//----------------------------------------ROS----------------------------------------//

// ------------------------------------//
// ROS for this UI
// -----------------------------------//

var sub_receieve_data = new ROSLIB.Topic({
	ros:ros,
	name: '/receieve_data',
	messageType : 'std_msgs/UInt8'
});

sub_receieve_data.subscribe(function(msg)
{
	console.log('click receieve_data');
	updata_feed_back_display(msg.data)
});

function updata_feed_back_display(receieve_data_msg)
{
	if(receieve_data_msg==1)
		document.getElementById("DisplayTableimg").src = "img/number1.png";
	else if(receieve_data_msg==2)
		document.getElementById("DisplayTableimg").src = "img/number2.png";
	else if(receieve_data_msg==3)
		document.getElementById("DisplayTableimg").src = "img/number3.png";
	else if(receieve_data_msg==0)
		document.getElementById("DisplayTableimg").src = "img/system.png";
}


var gripper_control_client = new ROSLIB.Service({
    ros : ros,
    name : 'gripper_service',
    serviceType : 'comm_stm32/gripper_cmd'
});

var GRIPPER_CMD_WAIT_TIME = 3000
function send_gripper_cmd(id)
{
	var request = new ROSLIB.ServiceRequest
	({
		val : id,
	});

	gripper_control_client.callService(request, function (res) {
		var is_success = res.success;
	});

	if(id!=0) //stop
	{
		// wait 'GRIPPER_CMD_WAIT_TIME' ms, then run function -> $("#btn_stop_all").click(); 
		setTimeout(function(){$("#btn_stop_all").click();}, GRIPPER_CMD_WAIT_TIME);  
	}
}

// ================== catch or loosen ==================
$("#btn_Catch_all").click(function()
{
	send_gripper_cmd(1)
    console.log('click btn_Catch_all');
});

$("#btn_Loosen_all").click(function()
{
	send_gripper_cmd(2)
    console.log('click btn_Loosen_all');
});

// ================== rotate gripper base ==================

$("#btn_rot2norm").click(function()
{
	send_gripper_cmd(10)
    console.log('click btn_rot2norm');
});

$("#btn_rot2parll").click(function()
{
	send_gripper_cmd(11)
    console.log('click btn_rot2parll');
});

$("#btn_rot2two_finger").click(function()
{
	send_gripper_cmd(12)
    console.log('click btn_rot2two_finger');
});

// ================== Control single finger ==================
$("#btn_catch_finger1").click(function()
{
	send_gripper_cmd(3)
    console.log('click btn_catch_finger1');
});

$("#btn_catch_finger2").click(function()
{
	send_gripper_cmd(4)
    console.log('click btn_catch_finger2');
});

$("#btn_catch_finger3").click(function()
{
	send_gripper_cmd(5)
    console.log('click btn_catch_finger3');
});

$("#btn_loosen_finger1").click(function()
{
	send_gripper_cmd(6)
    console.log('click btn_loosen_finger1');
});

$("#btn_loosen_finger2").click(function()
{
	send_gripper_cmd(7)
    console.log('click btn_loosen_finger2');
});

$("#btn_loosen_finger3").click(function()
{
	send_gripper_cmd(8)
    console.log('click btn_loosen_finger3');
});

// ================== Stop / Disable finger ==================
$("#btn_stop_all").click(function()
{
	send_gripper_cmd(0)
    console.log('click btn_stop_all');
});

$("#btn_disable_all").click(function()
{
	send_gripper_cmd(9)
    console.log('click btn_disable_all');
});
$("#btn_Connect").click(function()
{
	location.reload(true);
    console.log('click connect');
});
