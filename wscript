## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):    
#    obj = bld.create_ns3_program('vanet-highway-test', ['core'])
#    obj.source = [
#	'vanet-highway-test.cc',
#	'Highway.cc',
#	'Controller.cc',
#	'Vehicle.cc',
#	'Obstacle.cc',
#	'Model.cc',
#	'LaneChange.cc',
#	]

    obj = bld.create_ns3_program('20110920-review', ['core'])
    obj.source = [
	'20110920-review.cc',
	'Highway.cc',
	'Vehicle.cc',
	'Model.cc',
	'LaneChange.cc',
	]

