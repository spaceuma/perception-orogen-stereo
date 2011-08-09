#! /usr/bin/env ruby

require 'vizkit'
include Orocos

if ARGV.size < 1
    puts "usage: log_2_png.rb input_log outdir"
    exit 0
end

log = Orocos::Log::Replay.open( ARGV[0] )

log.camera_left.frame.connect_to do |data, ts|
    path = File.join( ARGV[1], "left_#{data.time.to_f}.png" )
    Vizkit.default_loader.ImageView.save_frame(data, path)
    data
end

log.camera_right.frame.connect_to do |data, ts|
    path = File.join( ARGV[1], "right_#{data.time.to_f}.png" )
    Vizkit.default_loader.ImageView.save_frame(data, path)
    data
end

Vizkit.control log
Vizkit.exec
