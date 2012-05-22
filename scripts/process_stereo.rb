#! /usr/bin/env ruby

require './config'
require 'vizkit'
require 'optparse'
include Orocos

def usage
    STDERR.puts 
    exit 1
end

calibration = :wide
batch_mode = false
batch_speed = nil 

opt_parse = OptionParser.new do |opt|
    opt.banner = "process_stereo.rb [-m calibration_file|-s calibration_symbol] <log_file|log_file_dir>"
    opt.on("-m calibration_file", String, "Filename of the stereo camera calibration from the Matlab Toolbox") do |name|
	calibration = name
    end
    opt.on("-s calibration_symbol", String, "Use build in calibration by the given name") do |name|
	calibration = name.to_sym
    end
    opt.on("-b speed", Float, "Use batch mode at given speed") do |name|
	batch_mode = true
	batch_speed = name
    end
end

args = opt_parse.parse( ARGV )
if args.size < 1
    puts opt_parse
    exit 1
end

if not args.empty?
    log_files = args
else
    log_files = Dir.glob(File.join("**","*[0-9].log"))
end

puts "Found #{log_files.size} logfiles"

log_files.each_with_index do |log_file,index|
  puts
  puts
  puts "Starting to convert #{log_file}"

  # filter out properties.log files
  if File.directory? log_file
      log_array = Dir.glob( File.join( log_file, "*.log" ) ).reject{|v| v.include? "properties"}
      log = Orocos::Log::Replay.open( log_array )
  else
      log = Orocos::Log::Replay.open( log_file )
  end
  
  Orocos::CORBA.max_message_size = 8000000

  Orocos.initialize
  
  Orocos::Process.run 'stereo_test', 'valgrind'=>false, "wait" => 1 do |p|

    stereo = p.task('stereo')

    log.camera_left.frame.connect_to stereo.left_frame #, :type => :buffer, :size => 1
    log.camera_right.frame.connect_to stereo.right_frame #,:type => :buffer, :size => 1
    
    # only generate the output log in batch mode
    if batch_mode 
	log_dir = if File.directory? log_file then log_file else File.dirname log_file end
	Orocos.log_all_ports( {:log_dir => log_dir} )
    end

    # configure the camera calibration
    stereo.stereoCameraCalibration do |stereoCamCal|
	cal_config( calibration, stereoCamCal )
    end
    
    # configure libElas
    libElas_conf = stereo.libElas_conf
    libElas_conf.disp_min              = 5 #^= distance up to 14m
    libElas_conf.disp_max              = 400 #^= 0.1757m
    libElas_conf.support_threshold     = 0.9
    libElas_conf.support_texture       = 10
    libElas_conf.candidate_stepsize    = 3 #3,5
    libElas_conf.incon_window_size     = 5
    libElas_conf.incon_threshold       = 5
    libElas_conf.incon_min_support     = 5
    libElas_conf.add_corners           = false
    libElas_conf.grid_size             = 20

    #values out of the paper
    libElas_conf.beta                  = 0.03
    libElas_conf.gamma                 = 15
    libElas_conf.sigma                 = 3
    
#     libElas_conf.beta                  = 0.02
#     libElas_conf.gamma                 = 3
#     libElas_conf.sigma                 = 1

    libElas_conf.sradius               = 2    
    libElas_conf.match_texture         = 1 #0,1
    libElas_conf.lr_threshold          = 1 #innen auch 1
    libElas_conf.speckle_sim_threshold = 2
    libElas_conf.speckle_size          = 350
    libElas_conf.ipol_gap_width        = 0 #vorher:5
    libElas_conf.filter_median         = false
    libElas_conf.filter_adaptive_mean  = false
    libElas_conf.postprocess_only_left = false
    libElas_conf.subsampling           = false
    stereo.libElas_conf = libElas_conf

    stereo.sparse_config do |c|
	c.knn = 2
	c.distanceFactor = 1.8 
	c.isometryFilterMaxSteps = 1000
	c.isometryFilterThreshold = 0.1
	c.detectorType = :DETECTOR_SURF
    end
    
    stereo.configure
    stereo.start

    if batch_mode 
	log.run(true, batch_speed)
    else
	# start the vizkit gui interface
	widget = Vizkit.default_loader.create_widget("vizkit::Vizkit3DWidget")
	vizkit_stereo = widget.createPlugin("stereo", "DistanceImageVisualization")
	widget.show

	# collect the stereo images from the output port
	stereo.distance_frame.connect_to do |data, name|
	    vizkit_stereo.updateDistanceImage data if data
	    data
	end

	Vizkit.display stereo.disparity_frame
	Vizkit.display log.camera_left.frame
	Vizkit.display stereo.sparse_debug
	Vizkit.control log

	Vizkit.exec
    end
  end
end
