#!/usr/bin/env python3
from cereal import car
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import RadarInterfaceBase
from selfdrive.car.hyundai.values import DBC


def get_radar_can_parser(CP):
  signals = [
    # sig_name, sig_address, default
    ("ACC_ObjStatus", "SCC11", 0),
    ("ACC_ObjLatPos", "SCC11", 0),
    ("ACC_ObjDist", "SCC11", 0),
    ("ACC_ObjRelSpd", "SCC11", 0),
  ]
  checks = [
    # address, frequency
    ("SCC11", 50),
  ]
  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, CP.sccBus)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.rcp = get_radar_can_parser(CP)
    self.updated_messages = set()
    self.trigger_msg = 0x420
    self.track_id = 0
    self.radar_off_can = CP.sccBus == -1

  def update(self, can_strings):
    if self.radar_off_can:
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return car.RadarData.new_message()

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = car.RadarData.new_message()
    cpt = self.rcp.vl
    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    if cpt["SCC11"]['ACC_ObjDist'] < 150:
      self.pts[0] = car.RadarData.RadarPoint.new_message()
      self.pts[0].trackId = self.track_id
      self.pts[0].dRel = cpt["SCC11"]['ACC_ObjDist']  # from front of car
      self.pts[0].yRel = -cpt["SCC11"]['ACC_ObjLatPos']  # in car frame's y axis, left is negative
      self.pts[0].vRel = cpt["SCC11"]['ACC_ObjRelSpd']
      self.pts[0].aRel = float('nan')
      self.pts[0].yvRel = float('nan')
      self.pts[0].measured = bool(cpt["SCC11"]['ACC_ObjStatus'])

    ret.points = list(self.pts.values())
    return ret

