from flange import *
from bolt import *

def test_part_flange():
    fl_up = {
        'name': 'fl_up',
        'tube_rad': 4300 / 2.0,
        'fl_width': 300,
        'fl_thick': 160,
        'tube_height': 300,
        'tube_thick': 40,
        'bolt_num': 40,
        'bolthole_offset': 120,
        'washer_rad': 52.5,
        'contact_thick': 10,
        'bolthole_rad': 30,
        'seedsize': None,
        'shell_seed_num': 4,
        'inner': True,
        'shell_height': 1000,
        'ref_point': True,
    }
    print("fl_up", fl_up)
    part_flange(**fl_up)

def test_assemble_flange():
    fl_assembly = {
        'name': 'myflange',
        'tube_rad': 4300 / 2.0,
        'tube_thick': 40,
        'tube_height': 300,
        'fl_width': 300,
        'fl_thick': 160,
        'bolt_num': 108,
        'bolthole_offset': 120,
        'washer_rad': 52.5,
        'contact_thick': 10,
        'bolthole_rad': 30,
        'bolt_rad': 25.42,
        'washer_thick': 10,
        'hex_circrad': 93.56 / 2.0 * 0.93,
        'hex_height': 35,
        'bolt_prestress': 1.28e6,
        'seedsize': None,
        'shell_height': 2000,
        'loading': [5e6, 1e6, 8.5e10, 1.6e9],
    }
    assemble_flange(**fl_assembly)

if __name__ == "__main__":
    import sys

    cases = {
        '1': test_part_flange,
        '2': test_assemble_flange,
    }

    cases[sys.argv[-1]]()