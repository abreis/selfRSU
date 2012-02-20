## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):    
    obj = bld.create_ns3_program('20120220-2laneNoRadio', ['core', 'mobility', 'wifi', 'applications', 'tools'])
    obj.source = [
	'20120220-2laneNoRadio.cc',
	'Highway.cc',
	'Vehicle.cc',
	'Model.cc',
	'LaneChange.cc',
	'VanetHeader.cc'
	]

