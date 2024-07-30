from opendbc.can.parser import CANParser
from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.volkswagen.values import DBC, VolkswagenFlags
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase

RADAR_ADDR = 0x24F
RADAR_SAME_LANE_01 = 1
RADAR_SAME_LANE_02 = 2

def get_radar_can_parser(CP):
  if CP.flags & VolkswagenFlags.MEB:
    messages = [("MEB_Distance_01", 25)]
  else:
    return None

  return CANParser(DBC[CP.carFingerprint]['radar'], messages, 2)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages = set()
    self.trigger_msg = RADAR_ADDR
    self.track_id = 0

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = car.RadarData.new_message()
    if self.rcp is None:
      return ret

    errors = []

    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    msg = self.rcp.vl["MEB_Distance_01"]

    # ---------------------------------- #

    signal_part = RADAR_SAME_LANE_01
    if signal_part not in self.pts:
      self.pts[signal_part]         = car.RadarData.RadarPoint.new_message()
      self.pts[signal_part].trackId = self.track_id
      self.track_id                += 1

    valid = msg['Same_Lane_01_Detection'] > 0
    if valid:
      self.pts[signal_part].measured = True
      self.pts[signal_part].dRel     = max(msg['Same_Lane_01_Long_Distance'], 0) #because of a little bit of shifting negative values possible
      self.pts[signal_part].yRel     = -msg['Same_Lane_01_Lat_Distance'] # left is positive
      self.pts[signal_part].vRel     = msg['Same_Lane_01_Rel_Velo'] * CV.KPH_TO_MS
      self.pts[signal_part].aRel     = float('nan')
      self.pts[signal_part].yvRel    = float('nan')
    else:
      del self.pts[signal_part]

    # ---------------------------------- #

    #signal_part = RADAR_SAME_LANE_02
    #if signal_part not in self.pts:
    #  self.pts[signal_part]         = car.RadarData.RadarPoint.new_message()
    #  self.pts[signal_part].trackId = self.track_id
    #  self.track_id                += 1

    #valid = msg['Same_Lane_02_Detection'] > 0 and msg['Same_Lane_02_Long_Distance'] > 0
    #if valid:
    #  self.pts[signal_part].measured = True
    #  self.pts[signal_part].dRel     = msg['Same_Lane_02_Long_Distance']
    #  self.pts[signal_part].yRel     = -msg['Same_Lane_02_Lat_Distance']
    #  self.pts[signal_part].vRel     = float('nan')
    #  self.pts[signal_part].aRel     = float('nan')
    #  self.pts[signal_part].yvRel    = float('nan')
    #else:
    #  del self.pts[signal_part]

    # ---------------------------------- #

    ret.points = list(self.pts.values())
    return ret
