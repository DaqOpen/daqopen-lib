import unittest
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from daqopen.picodaq import PicoDaq

class TestDaqInfo(unittest.TestCase):
    def test_from_dict_to_dict(self):
        pass

if __name__ == "__main__":
    unittest.main()
