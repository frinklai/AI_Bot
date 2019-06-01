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
	if(receieve_data_msg==0)
		document.getElementById("DisplayTableimg").src = "img/idle.png";

	else if(receieve_data_msg==1)
		document.getElementById("DisplayTableimg").src = "img/catch.png";

	else if(receieve_data_msg==2)
		document.getElementById("DisplayTableimg").src = "img/loosen.png";

	else if(receieve_data_msg==3)
		document.getElementById("DisplayTableimg").src = "img/finger1_catch.png";

	else if(receieve_data_msg==4)
		document.getElementById("DisplayTableimg").src = "img/finger2_catch.png";

	else if(receieve_data_msg==5)
		document.getElementById("DisplayTableimg").src = "img/finger3_catch.png";

	else if(receieve_data_msg==6)
		document.getElementById("DisplayTableimg").src = "img/finger1_loosen.png";

	else if(receieve_data_msg==7)
		document.getElementById("DisplayTableimg").src = "img/finger2_loosen.png";

	else if(receieve_data_msg==8)
		document.getElementById("DisplayTableimg").src = "img/finger3_loosen.png";

	else if(receieve_data_msg==9)
		document.getElementById("DisplayTableimg").src = "img/system.png";

	else if(receieve_data_msg==10)
		document.getElementById("DisplayTableimg").src = "img/drag_mode.png";

	else if(receieve_data_msg==11) 
		document.getElementById("DisplayTableimg").src = "img/2D_mode.png";

	else if(receieve_data_msg==12)
		document.getElementById("DisplayTableimg").src = "img/3D_mode.png";
	
	else if(receieve_data_msg==13)
		document.getElementById("DisplayTableimg").src = "img/stop.png";

	else if(receieve_data_msg==14)
		document.getElementById("DisplayTableimg").src = "img/catch.png";
	
	else if(receieve_data_msg==15)
		document.getElementById("DisplayTableimg").src = "img/loosen.png";

}


var gripper_control_client = new ROSLIB.Service({
    ros : ros,
    name : 'gripper_service',
    serviceType : 'comm_stm32/gripper_cmd'
});

var CATCH_LOOSEN_WAIT_TIME  = 3000;
var BASE_ROTATION_WAIT_TIME = 2000;
function send_gripper_cmd(id)
{
	var request = new ROSLIB.ServiceRequest
	({
		val : id,
	});

	gripper_control_client.callService(request, function (res) {
		var is_success = res.success;
	});

	if((id!=0)&&(id!=5)&&(id!=15)) //stop
	// if(id!=0) //if id is not 0, delay some time then stop
	{
		// wait 'GRIPPER_CMD_WAIT_TIME' ms, then run function -> $("#btn_stop_all").click(); 
		if((id==10)||(id==12))
			setTimeout(function(){$("#btn_stop_all").click();}, BASE_ROTATION_WAIT_TIME);    // base rotation, wait 2s
		else
			if(id==5)
				console.log('aaaaa');
			setTimeout(function(){$("#btn_stop_all").click();}, CATCH_LOOSEN_WAIT_TIME);  	 // catch or loosen, wait 3s
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

$("#btn_2D_Catch").click(function()
{
	send_gripper_cmd(14) 
	console.log('click btn_2D_Catch');
});

$("#btn_2D_Loosen").click(function()
{
	send_gripper_cmd(15)  											 					// loosen 2 finger
	setTimeout(function(){$("#btn_stop_all").click();}, CATCH_LOOSEN_WAIT_TIME);  	 // catch or loosen, wait 3s
	console.log('click btn_2D_Loosen');
});

$("#btn_3D_catch_to_2D").click(function()
{
	send_gripper_cmd(5)  // when 											 					// Catch finger 3
	setTimeout(function(){$(send_gripper_cmd(14)).click();}, CATCH_LOOSEN_WAIT_TIME);  // Catch 2 finger
	console.log('click btn_2D_Catch');
});

$("#btn_2D_Loosen_to_3D").click(function()
{
	send_gripper_cmd(15)  											 					// loosen 2 finger
	setTimeout(function(){$(send_gripper_cmd(8)).click();}, CATCH_LOOSEN_WAIT_TIME);   // loosen finger 3
	console.log('click btn_2D_Loosen');
});

// --
// $("#btn_3D_catch_to_2D").click(function()
// {
// 	send_gripper_cmd(11);  	
// 	setTimeout(function(){$(send_gripper_cmd(5)).click();}, BASE_ROTATION_WAIT_TIME);
// 	setTimeout(function(){$(send_gripper_cmd(14)).click();}, BASE_ROTATION_WAIT_TIME+CATCH_LOOSEN_WAIT_TIME+1000);
// 	console.log('click btn_2D_Catch');
// });


// $("#btn_2D_Loosen_to_3D").click(function()
// {
// 	send_gripper_cmd(11)  // when 											 		   // rot to 2D_mode
// 	setTimeout(function(){$(send_gripper_cmd(8)).click();}, BASE_ROTATION_WAIT_TIME);  // loosen finger 3
// 	setTimeout(function(){$(send_gripper_cmd(15)).click();}, 6000);  // loosen 2 finger
// 	console.log('click btn_2D_Loosen');
// });

// ================== rotate gripper base ==================

$("#btn_rot2norm").click(function()			// 3D_mode
{
	send_gripper_cmd(12)
	console.log('click btn_rot2norm');
});

$("#btn_rot2parll").click(function() 		//drag_mode
{
	send_gripper_cmd(10)
	console.log('click btn_rot2parll');
});

$("#btn_rot2two_finger").click(function()	// 2D_mode
{
	send_gripper_cmd(11)
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
	console.log('click btn_catch_finger31');
	setTimeout(function(){$("#btn_stop_all").click();}, CATCH_LOOSEN_WAIT_TIME);  
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
	setTimeout(function(){$("#btn_stop_all").click();}, CATCH_LOOSEN_WAIT_TIME);  
    console.log('click btn_disable_all');
});
$("#btn_Connect").click(function()
{
	location.reload(true);
	console.log('click connect');
	document.getElementById("DisplayTableimg").src = "img/idle.png";
});
