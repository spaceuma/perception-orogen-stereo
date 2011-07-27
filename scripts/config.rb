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
    pp cal_name
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
    # configure dense_stereo from predefined configs
    elsif( cal_name == :wide)
	puts "wide calibration used"
      #asguard wide angle lens
      camLeft = stereoCamCal.camLeft
      #intrinsic parameters
      camLeft.fx = 284.24382
      camLeft.fy = 285.20982
      camLeft.cx = 323.63161
      camLeft.cy = 232.15181
      camLeft.d0 = 0.00343
      camLeft.d1 = 0.00094
      camLeft.d2 = -0.00069
      camLeft.d3 = 0.00145
      stereoCamCal.camLeft = camLeft
      
      camRight = stereoCamCal.camRight
      camRight.fx = 285.57255
      camRight.fy = 286.28520
      camRight.cx = 318.59641
      camRight.cy = 230.64484
      camRight.d0 = 0.00112
      camRight.d1 = 0.00244
      camRight.d2 = 0.00007
      camRight.d3 = 0.00010
      stereoCamCal.camRight = camRight
      
      extrinsic = stereoCamCal.extrinsic
      extrinsic.tx = -251.92827
      extrinsic.ty = -0.12599
      extrinsic.tz = -8.36759
      
      extrinsic.rx = -0.00462
      extrinsic.ry = -0.01025
      extrinsic.rz = 0.00650
      stereoCamCal.extrinsic = extrinsic
      
    else
	puts "normal calibration used"
      #asguard cam guppy mar 2011
      camLeft = stereoCamCal.camLeft
      #intrinsic parameters
      camLeft.fx = 701.60321
      camLeft.fy = 703.61811
      camLeft.cx = 324.00043
      camLeft.cy = 256.81324
      camLeft.d0 = -0.03207
      camLeft.d1 = 0.05658
      camLeft.d2 = -0.00060
      camLeft.d3 = 0.00207
      stereoCamCal.camLeft = camLeft
      
      camRight = stereoCamCal.camRight
      camRight.fx = 701.48689
      camRight.fy = 703.66743
      camRight.cx = 317.56601
      camRight.cy = 235.52183
      camRight.d0 = -0.04774
      camRight.d1 = 0.08554
      camRight.d2 = -0.00080
      camRight.d3 = -0.00071
      stereoCamCal.camRight = camRight
      
      extrinsic = stereoCamCal.extrinsic
      extrinsic.tx = -253.28725
      extrinsic.ty = -0.51154
      extrinsic.tz = -3.52484
      
      extrinsic.rx = -0.00202
      extrinsic.ry = -0.00352
      extrinsic.rz = 0.00712
      stereoCamCal.extrinsic = extrinsic
    end
end
