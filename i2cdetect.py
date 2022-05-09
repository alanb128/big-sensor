
#  py-i2cdetect.py
# 
#  Lists all the active addresses on the selected I2C bus.
#
#  This  program is free software: you can redistribute it and/or modify  it
#  under  the  terms of the GNU General Public License as published  by  the
#  Free  Software  Foundation, either version 3 of the License, or (at  your
#  option) any later version.
#
#  This  program  is  distributed in the hope that it will  be  useful,  but
#  WITHOUT   ANY   WARRANTY;   without  even   the   implied   warranty   of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
#  Public License for more details.
#
#  You  should have received a copy of the GNU General Public License  along
#  with this program.  If not, see <http://www.gnu.org/licenses/>.
# 
#  06 Jun 21   0.1   - Initial version - MT
#
import sys
import smbus

def _list_devices(_bus):
  sys.stdout.write ('   ')
  for _address in range(16):
    sys.stdout.write (' %2x' % _address) # Print header
  
  for _address in range(128):
    if not _address % 16:
      sys.stdout.write ('\n%02x:' % _address) # Print address
    if 2 < _address < 120 : # Skip reserved addresses
      try:
        _bus.read_byte(_address)
        sys.stdout.write (' %02x' % _address) # Device address
      except:
        sys.stdout.write (' --') # No device detected
    else:
      sys.stdout.write ('   ') # Reserved

  sys.stdout.write ('\n')

try:
  _list_devices(smbus.SMBus(1))
except KeyboardInterrupt:
  sys.stdout.write('\n')
except Exception as _error:
  sys.stderr.write('%s\n' % str(_error))
  sys.exit(1)
finally:
  sys.exit(0)