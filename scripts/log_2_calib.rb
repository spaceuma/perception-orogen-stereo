#! /usr/bin/env ruby

require 'vizkit'
include Orocos

if ARGV.size < 1
    puts "usage: log_2_calib.rb input_log outdir"
    exit 0
end

class StereoWidget < Qt::Widget
    slots 'save()'

    def initialize( parent = nil )
	super()

	@left = nil
	@right = nil
	@index = 0
    end

    def setLog( log )
	@value = Qt::Label.new("0")
	@count = Qt::Label.new("0")
	@button = Qt::PushButton.new("Pick")

	buttons = Qt::Widget.new()
	buttonLayout = Qt::GridLayout.new()
	buttonLayout.addWidget( @value, 0, 0 )
	buttonLayout.addWidget( @count, 1, 0 )
	buttonLayout.addWidget( @button, 2, 0 )
	buttons.setLayout( buttonLayout )

	gridLayout = Qt::GridLayout.new()
	gridLayout.addWidget( Vizkit.display( log.camera_left.frame ), 0, 0 )
	gridLayout.addWidget( Vizkit.display( log.camera_right.frame ), 0, 1 )
	gridLayout.addWidget( buttons, 0, 2 )
	gridLayout.setColumnStretch( 0, 10 )
	gridLayout.setColumnStretch( 1, 10 )

	setLayout( gridLayout )

	connect(@button, SIGNAL('clicked()'), self, SLOT('save()'))
    end

    def save
	puts "save pair with index #{@index}"

	path = File.join( ARGV[1], "left_#{@index}.png" )
	Vizkit.default_loader.ImageView.save_frame(@left, path)

	path = File.join( ARGV[1], "right_#{@index}.png" )
	Vizkit.default_loader.ImageView.save_frame(@right, path)

	@index += 1
	@count.text = @index.to_s
    end

    def update
	if @left and @right
	    @value.text = (@left.time - @right.time).to_s
	end
    end

    def updateLeft( frame )
	@left = frame
	update
    end

    def updateRight( frame )
	@right = frame
	update
    end
end

log = Orocos::Log::Replay.open( ARGV[0] )

widget = StereoWidget.new
widget.setLog log
widget.show

log.camera_left.frame.connect_to do |data, ts|
    widget.updateLeft( data )
    data
end

log.camera_right.frame.connect_to do |data, ts|
    widget.updateRight( data )
    data
end

Vizkit.control log
Vizkit.exec

