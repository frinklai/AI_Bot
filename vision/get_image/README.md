usage: 
	1.Find FLIR camera id:
		(1) open another terminal
		(2) $ rosrun pointgrey_camera_driver list_cameras 
	2.Open camera's launch file(this will NOT show any image, only enable)
		(1) open another terminal
		(2) $ roslaunch pointgrey_camera_driver camera.launch serial_number:=id_you_found
			ex:
				roslaunch pointgrey_camera_driver camera.launch serial_number:=18073764
	3.Set config(this will open a ui to set param and see image)
		(1) open another terminal
		(2) $ rqt
		(3) click 'Plugins' button on the top left 
		(4) click 'Visualiztion' --> 'Image View'
			and now you can see the picture that camera takes.
		(5) click 'Configuration' --> 'Dynamic Reconfigure' 
		(6) click 'camera' --> 'camera_nodelet' of Dynamic Reconfigure GUI
			and now you can chaneg the config of camera.
	4.Take picture
		(1) open another terminal
		(2) rosrun get_image Get_Image.py --Object_Name=Class_name_of_training_object
			ex: 
				rosrun get_image Get_Image.py --Object_Name=Cat
		(3) key 's' to save picture in dir 'Training_data', which locate to the same dir of 'Get_Image.py'

