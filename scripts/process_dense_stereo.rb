#! /usr/bin/env ruby

require 'config'
require 'vizkit'
require 'optparse'
include Orocos

def usage
    STDERR.puts 
    exit 1
end

calibration = :wide
batch_mode = false

opt_parse = OptionParser.new do |opt|
    opt.banner = "process_dense_stereo.rb [-m calibration_file|-s calibration_symbol] <log_file|log_file_dir>"
    opt.on("-m calibration_file", String, "Filename of the stereo camera calibration from the Matlab Toolbox") do |name|
	calibration = name
    end
    opt.on("-s calibration_symbol", String, "Use build in calibration by the given name") do |name|
	calibration = name.to_sym
    end
    opt.on("-b", String, "Use batch mode") do |name|
	batch_mode = true
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
      log_file = Dir.glob( File.join( log_file, "*.log" ) ).reject{|v| v.include? "properties"}
  end

  log = Orocos::Log::Replay.open(log_file)
  
  Orocos::CORBA.max_message_size = 8000000

  Orocos.initialize
  
  Orocos::Process.spawn 'dense_stereo_test', 'valgrind'=>false, "wait" => 1000 do |p|

    dense_stereo = p.task('dense_stereo')

    log.camera_left.frame.connect_to dense_stereo.left_frame, :type => :buffer, :size => 1
    log.camera_right.frame.connect_to dense_stereo.right_frame,:type => :buffer, :size => 1
    
    # only generate the output log in batch mode
    if batch_mode 
	log_dir = if File.directory? log_file then log_file else File.dirname log_file end
	Orocos.log_all_ports( {:log_dir => log_dir} )
    end

    # configure the camera calibration
    dense_stereo.stereoCameraCalibration do |stereoCamCal|
	cal_config( calibration, stereoCamCal )
    end
    
    # configure libElas
    libElas_conf = dense_stereo.libElas_conf
    libElas_conf.disp_min              = 4
    libElas_conf.disp_max              = 400
    libElas_conf.support_threshold     = 0.88
    libElas_conf.support_texture       = 10
    libElas_conf.candidate_stepsize    = 3 #5
    libElas_conf.incon_window_size     = 5
    libElas_conf.incon_threshold       = 5
    libElas_conf.incon_min_support     = 5
    libElas_conf.add_corners           = false
    libElas_conf.grid_size             = 20
    libElas_conf.beta                  = 0.02
    libElas_conf.gamma                 = 3
    libElas_conf.sigma                 = 1
    libElas_conf.sradius               = 2
    libElas_conf.match_texture         = 1 #0,1
    libElas_conf.lr_threshold          = 2 #vorher:5
    libElas_conf.speckle_sim_threshold = 1 #1.5,1
    libElas_conf.speckle_size          = 200
    libElas_conf.ipol_gap_width        = 3 #vorher:5
    libElas_conf.filter_median         = false
    libElas_conf.filter_adaptive_mean  = true
    libElas_conf.postprocess_only_left = false
    libElas_conf.subsampling           = false
    dense_stereo.libElas_conf = libElas_conf
    
    dense_stereo.configure
    dense_stereo.start

    if batch_mode 
	log.run(true, 1)
    else
	# start the vizkit gui interface
	widget = Vizkit.default_loader.create_widget("vizkit::QVizkitWidget")
	vizkit_dense_stereo = widget.createPlugin("DistanceImageVisualization", "dense_stereo")

	# collect the stereo images from the output port
	dense_stereo.distance_frame.connect_to do |data, name|
	    vizkit_dense_stereo.updateDistanceImage data if data
	    data
	end

	Vizkit.display dense_stereo.disparity_frame
	Vizkit.display log.camera_left.frame
	Vizkit.control log

	widget.show

	Vizkit.exec
    end
  end
end
