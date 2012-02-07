## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):    
    obj = bld.create_ns3_program('20120207-noRadioModel', ['core', 'mobility', 'wifi', 'applications', 'tools'])
    obj.source = [
	'20120207-noRadioModel.cc',
	'Highway.cc',
	'Vehicle.cc',
	'Model.cc',
	'LaneChange.cc',
	'VanetHeader.cc'
	]

