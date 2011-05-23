#! /usr/bin/env ruby

require 'vizkit'
include Orocos

  ts1 = ''
  ts2 = ''

  initial_offset = 0
  last_offset = 0
  
  calibration = :wide #for wide lens calibration
  #calibration = :normal #for normal lens calibration

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
  
  Orocos::Process.spawn 'dense_stereo_test', 'valgrind'=>false, "wait" => 1000 do |p|

    dense_stereo = p.task('dense_stereo')

    log.camera_left.frame.connect_to dense_stereo.left_frame, :type => :buffer, :size => 1
    log.camera_right.frame.connect_to dense_stereo.right_frame,:type => :buffer, :size => 1
    
    Orocos.log_all_ports

    reader = dense_stereo.distance_frame.reader(:type => :buffer, :size => 1)
    counter = 0

    # configure the camera calibration
    dense_stereo.stereoCameraCalibration do |stereoCamCal|
	cal_config( calibration, stereoCamCal )
    end
    
    # configure libElas
    libElas_conf = dense_stereo.libElas_conf
    libElas_conf.postprocess_only_left = true
    dense_stereo.libElas_conf = libElas_conf
    
    #set the pixel size of the camera
    dense_stereo.cameraPixelSize_x = 6.4*10**-6
    dense_stereo.cameraPixelSize_y = 6.4*10**-6
    
    dense_stereo.configure
    dense_stereo.start
    
    while log.step(false) != nil
      frame1 = reader.read_new
      
      timeout = 0
      
      while frame1 == nil && timeout < 15
	frame1 = reader.read_new
	sleep(0.3)
	timeout += 1
      end
      
      next if frame1 == nil
      
      #t1 = frame1.time
      #ts1 = t1.tv_sec * 1000000 + t1.tv_usec

      #path = "#{log_file}_#{reader.port.task.name}_#{reader.port.name}#{counter}.png"
	#puts path
    #  Vizkit.default_loader.ImageView.save_frame(frame1,path)

    #  counter += 1

      GC.start
    end
  end

end
