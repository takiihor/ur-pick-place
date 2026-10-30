"""Exercise the real frontend and HTTP workers in an isolated headless browser."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import unittest

import vision_sim  # noqa: F401 -- skip before importing optional vision code


class ValidationBrowserTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('node') and shutil.which('google-chrome'),
                         'Node and Google Chrome are needed for browser integration')
    def test_step_five_with_real_http_and_capture_workers(self):
        script = Path(__file__).with_name('marker_validation_browser.cjs')
        result = subprocess.run(['node', str(script)], capture_output=True, text=True, timeout=110,
            env=dict(os.environ, PYTHON=sys.executable, CHROME_BINARY=shutil.which('google-chrome')))
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertIn('Browser verified:', result.stdout)


if __name__ == '__main__':
    unittest.main()
