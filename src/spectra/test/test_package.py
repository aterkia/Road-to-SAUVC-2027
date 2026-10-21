import unittest

from spectra.node import main


class PackageTest(unittest.TestCase):
    def test_installed_entry_point(self):
        self.assertTrue(callable(main))
