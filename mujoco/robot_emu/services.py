"""Sport RPC responder: answers rt/api/sport/request like the real dog's
sport service. Request/response are plain unitree_api topics (no lease for
the sport family -- the client calls with lease disabled); go2's Move is a
no-reply call on the client side, an answer is written anyway and simply
ignored there.

Runs handlers on the SDK reader thread: never raise out of _on_request.
"""
from __future__ import annotations

import json
import traceback

from . import fsm as F

REQ_TOPIC = "rt/api/sport/request"
RESP_TOPIC = "rt/api/sport/response"


class SportService:
    def __init__(self, fsm: F.EmuFSM, cmd_source=None, scenarios=None) -> None:
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_api.msg.dds_ import Request_, Response_, \
            ResponseHeader_, RequestIdentity_, ResponseStatus_

        self._fsm = fsm
        self._cmd = cmd_source
        self._sc = scenarios
        self._Response = Response_
        self._Header = ResponseHeader_
        self._Identity = RequestIdentity_
        self._Status = ResponseStatus_
        self._n_req = 0
        self._pub = ChannelPublisher(RESP_TOPIC, Response_)
        self._pub.Init()
        self._sub = ChannelSubscriber(REQ_TOPIC, Request_)
        self._sub.Init(self._on_request, 10)

    @property
    def n_requests(self) -> int:
        return self._n_req

    def _on_request(self, req) -> None:
        try:
            self._n_req += 1
            api_id = int(req.header.identity.api_id)
            code = self._fsm.handle(api_id)
            if self._cmd is not None:
                if api_id == F.MOVE:
                    # scenario gate: obstacle refusal / cmd blackout refuse
                    # the motion and drop the held setpoint, like a chassis
                    # that stopped executing.
                    if code == 0 and self._sc is not None and (
                            self._sc.get("refuse_move")
                            or self._sc.blackout_active()):
                        code = -1
                    try:
                        p = json.loads(req.parameter) if req.parameter else {}
                    except json.JSONDecodeError:
                        p = {}
                    if code == 0:
                        code = self._cmd.accept(
                            float(p.get("x", 0.0)), float(p.get("y", 0.0)),
                            float(p.get("z", 0.0)), code)
                    else:
                        self._cmd.stop_move()
                elif api_id == F.STOP_MOVE and code == 0:
                    self._cmd.stop_move()
            data = ""
            if api_id == F.GET_STATE:
                data = json.dumps({"mode": self._fsm.mode})
            header = self._Header(
                self._Identity(int(req.header.identity.id), api_id),
                self._Status(int(code)))
            self._pub.Write(self._Response(header, data, []))
        except Exception:      # noqa: BLE001 -- reader thread must stay alive
            traceback.print_exc()
