## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):    
    obj = bld.create_ns3_program('20110921-clean', ['core', 'mobility', 'wifi', 'applications', 'tools'])
    obj.source = [
	'20110921-clean.cc',
	'Highway.cc',
	'Vehicle.cc',
	'Model.cc',
	'LaneChange.cc',
	]

