#! /usr/bin/env ruby

require 'vizkit'
include Orocos

  ts1 = ''
  ts2 = ''

  initial_offset = 0
  last_offset = 0
  
  calibration = 1 #for wide lens calibration
  #calibration = 0 #for normal lens calibration

if !ARGV.empty?
  log_files = ARGV
else
  log_files = Dir.glob(File.join("**","*[0-9].log"))
end

puts "Found #{log_files.size} logfiles"

log_files.each_with_index do |log_file,index|
  puts
  puts
  puts "Starting to convert #{log_file}"
  log = Orocos::Log::Replay.open(log_file)
  
  Orocos::CORBA.max_message_size = 8000000

  Orocos.initialize
  
  Orocos.run 'dense_stereo_test' do
  
  dense_stereo = Orocos::TaskContext.get 'dense_stereo'

  log.camera_left.frame.connect_to dense_stereo.left_frame, :type => :buffer, :size => 1
  log.camera_right.frame.connect_to dense_stereo.right_frame,:type => :buffer, :size => 1
  
  Orocos.log_all_ports

  #ports = dense_stereo.find_all_ports('/base/samples/frame/Frame',nil)
#"/_disparity_/"
  #puts "  Found #{ports.size} port(s) of type /base/samples/frame/Frame"
  
  #for each port
  readers = Array.new
  counter = Array.new
  #ports.each do |port|
  #  puts port.name
  #  readers << port.reader
  #  counter << 0
  #end

  readers << dense_stereo.left_disparity_frame.reader(:type => :buffer, :size => 1)
  counter << 0
  readers << dense_stereo.right_disparity_frame.reader(:type => :buffer, :size => 1)
  counter << 0
  
  #configure dense_stereo
  if(calibration == 1)
    puts "wide calibration used"
  #asguard wide angle lens
  stereoCamCal = dense_stereo.stereoCameraCalibration
  camLeft = stereoCamCal.CamLeft
  #intrinsic parameters
  camLeft.fx = 284.24382
  camLeft.fy = 285.20982
  camLeft.cx = 323.63161
  camLeft.cy = 232.15181
  camLeft.d0 = 0.00343
  camLeft.d1 = 0.00094
  camLeft.d2 = -0.00069
  camLeft.d3 = 0.00145
  stereoCamCal.CamLeft = camLeft
  
  camRight = stereoCamCal.CamRight
  camRight.fx = 285.57255
  camRight.fy = 286.28520
  camRight.cx = 318.59641
  camRight.cy = 230.64484
  camRight.d0 = 0.00112
  camRight.d1 = 0.00244
  camRight.d2 = 0.00007
  camRight.d3 = 0.00010
  stereoCamCal.CamRight = camRight
  
  extrinsic = stereoCamCal.extrinsic
  extrinsic.tx = -251.92827
  extrinsic.ty = -0.12599
  extrinsic.tz = -8.36759
  
  extrinsic.rx = -0.00462
  extrinsic.ry = -0.01025
  extrinsic.rz = 0.00650
  stereoCamCal.extrinsic = extrinsic
  
  dense_stereo.stereoCameraCalibration = stereoCamCal
  else
    puts "normal calibration used"
  #asguard cam guppy mar 2011
  stereoCamCal = dense_stereo.stereoCameraCalibration
  camLeft = stereoCamCal.CamLeft
  #intrinsic parameters
  camLeft.fx = 701.60321
  camLeft.fy = 703.61811
  camLeft.cx = 324.00043
  camLeft.cy = 256.81324
  camLeft.d0 = -0.03207
  camLeft.d1 = 0.05658
  camLeft.d2 = -0.00060
  camLeft.d3 = 0.00207
  stereoCamCal.CamLeft = camLeft
  
  camRight = stereoCamCal.CamRight
  camRight.fx = 701.48689
  camRight.fy = 703.66743
  camRight.cx = 317.56601
  camRight.cy = 235.52183
  camRight.d0 = -0.04774
  camRight.d1 = 0.08554
  camRight.d2 = -0.00080
  camRight.d3 = -0.00071
  stereoCamCal.CamRight = camRight
  
  extrinsic = stereoCamCal.extrinsic
  extrinsic.tx = -253.28725
  extrinsic.ty = -0.51154
  extrinsic.tz = -3.52484
  
  extrinsic.rx = -0.00202
  extrinsic.ry = -0.00352
  extrinsic.rz = 0.00712
  stereoCamCal.extrinsic = extrinsic
  
  dense_stereo.stereoCameraCalibration = stereoCamCal
  end
  
  libElas_conf = dense_stereo.libElas_conf
  libElas_conf.postprocess_only_left = false
  dense_stereo.libElas_conf = libElas_conf
  
  dense_stereo.configure
  dense_stereo.start
  
  #puts readers[0].port.task.name
  #puts readers[1].port.task.name
  

      while log.step(false) != nil && log.step(false) != nil
	reader1 = readers[0]
	reader2 = readers[1]
	frame1 = reader1.read_new
	frame2 = reader2.read_new
	
	timeout = 0
	
	while frame1 == nil && timeout < 15
	  frame1 = reader1.read_new
	  sleep(0.3)
	  timeout += 1
	end
	
	next if frame1 == nil
	
	timeout = 0  
	
	while frame2 == nil && timeout < 15
	  frame2 = reader2.read_new
	  sleep(0.3)
	  timeout += 1
	end
	
	next if frame2 == nil
	
	t1 = frame1.time
	ts1 = t1.tv_sec * 1000000 + t1.tv_usec
 
	t2 = frame2.time
	ts2 = t2.tv_sec * 1000000 + t2.tv_usec


	puts "ts1 = " + ts1.to_s + "  ts2 = " + ts2.to_s
	offset = ts2 - ts1
        
	if initial_offset == 0
	   initial_offset = offset 
	   last_offset = offset
	end

	if offset.abs <= 1000
	    puts 'offset is ' + offset.to_s + ' microsecs and last offset was ' + last_offset.to_s
	    puts 'offset diff OK, now saving img pair to '
	  path1 = "#{log_file}_#{reader1.port.task.name}_#{reader1.port.name}#{counter[index]}.png"
	    puts path1
	  Vizkit.default_loader.ImageView.save_frame(frame1,path1)

	  path2 = "#{log_file}_#{reader2.port.task.name}_#{reader2.port.name}#{counter[index]}.png"
	    puts path2
	  Vizkit.default_loader.ImageView.save_frame(frame2,path2)
    
	  counter[index] += 1

	  last_offset = offset
	end
	GC.start
      end
  end

end