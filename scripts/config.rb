require 'pp'

class MatlabCalibration
    def initialize
	@params = Hash.new
    end

    def read( file_name )
	File.open( file_name, 'r' ) do |f|
	    while (line = f.gets)
		if line =~ /(\w+) = \[([^\]]+)\]/
		    @params[$1] = $2.split(" ").map! &:to_f
		end	    
	    end
	end
    end

    def pretty_print( pp )
	pp @params
    end

    def method_missing( name )
	@params[name.to_s]
    end
end

def cal_config( cal_name, stereoCamCal )
    # see if we need to load the calibration configuration from file
    if cal_name.is_a? String and File.exists? cal_name
	calib = MatlabCalibration.new
	calib.read cal_name
	puts "using config from file #{cal_name}"
	pp calib

	camLeft = stereoCamCal.camLeft
	camLeft.fx, camLeft.fy = calib.fc_left 
	camLeft.cx, camLeft.cy = calib.cc_left
	camLeft.d0, camLeft.d1, camLeft.d2, camLeft.d3 = calib.kc_left
	stereoCamCal.camLeft = camLeft

	camRight = stereoCamCal.camRight
	camRight.fx, camRight.fy = calib.fc_right 
	camRight.cx, camRight.cy = calib.cc_right
	camRight.d0, camRight.d1, camRight.d2, camRight.d3 = calib.kc_right
	stereoCamCal.camRight = camRight

	extrinsic = stereoCamCal.extrinsic
	extrinsic.tx, extrinsic.ty, extrinsic.tz = calib.T
	extrinsic.rx, extrinsic.ry, extrinsic.rz = calib.om
	stereoCamCal.extrinsic = extrinsic
    else
	raise "Could not open #{cal_name} for calibration"
    end
end
