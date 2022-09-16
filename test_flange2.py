from flange2 import *

def test_create_flange():
    fl_up = {
        'name': 'fl_up',
        'tube_diam': 4300 ,
        'tube_thick': 40,
        'tube_height': 300,
        'fl_width': 300,
        'fl_thick': 160,
        'bolt_num': 108,
        'bolthole_diam': 60,
        # 'bolthole_offset': 120,
        'boltcirc_diam': 4060,
        'washer_diam': 105,
        # 'flange_contact_thick': 10,
        'seedsize': 30,
        # 'shell_seed_num': 4,
        # 'flip': False,
        'inner': True,
        # 'draft': None,
        # 'prelim': False,
        # 'quarter': False,
        # 'hole_on_axis': True,
    }
    create_flange(**fl_up)

def test_assemble_flange():
    fl_up = {
        'name': 'fl_up',
        'tube_diam': 4300 ,
        'tube_thick': 40,
        'tube_height': 300,
        'fl_width': 300,
        'fl_thick': 160,
        'bolt_num': 108,
        'bolthole_diam': 60,
        # 'bolthole_offset': 120,
        'boltcirc_diam': 4060,
        'washer_diam': 105,
        # 'flange_contact_thick': 10,
        'seedsize': 30,
        # 'shell_seed_num': 4,
        # 'flip': False,
        'inner': True,
        # 'draft': None,
        # 'prelim': False,
        # 'quarter': False,
        # 'hole_on_axis': True,
    }
    fl_low = {
        'name': 'fl_up',
        'tube_diam': 4300 ,
        'tube_thick': 40,
        'tube_height': 300,
        'fl_width': 300,
        'fl_thick': 160,
        'bolt_num': 108,
        'bolthole_diam': 60,
        # 'bolthole_offset': 120,
        'boltcirc_diam': 4060,
        'washer_diam': 105,
        # 'flange_contact_thick': 10,
        'seedsize': 30,
        # 'shell_seed_num': 4,
        # 'flip': False,
        'inner': True,
        # 'draft': None,
        # 'prelim': False,
        # 'quarter': False,
        # 'hole_on_axis': True,
    }

if __name__ == '__main__':
    test_create_flange()