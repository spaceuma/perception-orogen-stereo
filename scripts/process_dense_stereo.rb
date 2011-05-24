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

opt_parse = OptionParser.new do |opt|
    opt.banner = "process_dense_stereo.rb [-m calibration_file|-s calibration_symbol] <log_file|log_file_dir>"
    opt.on("-m calibration_file", String, "Filename of the stereo camera calibration from the Matlab Toolbox") do |name|
	calibration = name
    end
    opt.on("-s calibration_symbol", String, "Use build in calibration by the given name") do |name|
	calibration = name.to_sym
    end
end

args = opt_parse.parse( ARGV )
if args.size < 1
    puts opt_parse
    exit 1
end

if not args.empty?
    log_files = args[0]
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

    #reader = dense_stereo.distance_frame.reader(:type => :buffer, :size => 1)
    #counter = 0

    # configure the camera calibration
    dense_stereo.stereoCameraCalibration do |stereoCamCal|
	cal_config( calibration, stereoCamCal )
    end
    
    # configure libElas
    libElas_conf = dense_stereo.libElas_conf
    libElas_conf.postprocess_only_left = true
    dense_stereo.libElas_conf = libElas_conf
    
    #set the pixel size of the camera
    dense_stereo.cameraPixelWidth = 6.4*10**-6
    dense_stereo.cameraPixelHeight = 6.4*10**-6
    
    dense_stereo.configure
    dense_stereo.start

    # start the vizkit gui interface
    widget = Vizkit.default_loader.create_widget("vizkit::QVizkitWidget")
    vizkit_dense_stereo = widget.createPlugin("DistanceImageVisualization", "dense_stereo")
    
    # collect the stereo images from the output port
    dense_stereo.distance_frame.connect_to do |data, name|
	vizkit_dense_stereo.updateDistanceImage data if data
	data
    end

    Vizkit.display log.camera_left.frame
    Vizkit.control log

    widget.show

    Vizkit.exec
  end
end
