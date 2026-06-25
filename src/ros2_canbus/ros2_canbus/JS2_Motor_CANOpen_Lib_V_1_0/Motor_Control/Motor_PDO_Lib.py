import canopen, re
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Logger_Lib import Logger
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Common_Lib import Common

class PDO_Lib:
    def __init__(self, node: canopen.RemoteNode, node_name: str):
        self.node = node
        self.Node_ID = self.node.id
        self.Node_Name = node_name
        self.common = Common(self.Node_ID, self.Node_Name)
        self.log = Logger(self.Node_Name, self.Node_ID)

    def configure_pdo_attempt_multiple(self,
                node_id: int,
                pdo_name: str,                 # "RPDO1".."RPDO4" or "TPDO1".."TPDO4" (spaces ok)
                variables,                      # ["Target_position", ("Profile_velocity", 0)], etc.
                trans_type: int | None = None,  # default: RPDO=254, TPDO=255
                event_timer: int | None = None, # ms; mostly for TPDOs
                enabled: bool = True):
        
        for i in range(1,5):
            self.log.print(f"PDO Configuration attempt {i}...", "INFO", "PDO_Lib", "configure_attempt_multiple", pdo_name)

            pdo = self.configure(
                                node_id=node_id,
                                pdo_name=pdo_name,
                                variables=variables,
                                trans_type=trans_type,
                                event_timer=event_timer,
                                enabled=enabled)
            self.common.delay_SDO()
            if pdo is not None:
                return pdo
        return None

    def configure(self,
                node_id: int,
                pdo_name: str,                 # "RPDO1".."RPDO4" or "TPDO1".."TPDO4" (spaces ok)
                variables,                      # ["Target_position", ("Profile_velocity", 0)], etc.
                trans_type: int | None = None,  # default: RPDO=254, TPDO=255
                event_timer: int | None = None, # ms; mostly for TPDOs
                enabled: bool = True):
        """
        Configure any RPDO/TPDO and return the PDO object on success.
        After saving, reads back the device PDO and verifies it matches the request.
        Returns None if the verification mismatches or any error occurs.
        """

        try:
            if self.node is None:
                raise RuntimeError("Node not initialized. Call add_node_to_network() first.")

            # ---------- normalize & parse PDO name ----------
            name = re.sub(r"\s+", "", str(pdo_name).upper())  # strip spaces, uppercase
            m = re.fullmatch(r'(R|T)PDO([1-4])', name)
            if not m:
                raise ValueError(f"Invalid pdo_name '{pdo_name}'. Use 'RPDO1'..'RPDO4' or 'TPDO1'..'TPDO4'.")

            kind = m.group(1)            # 'R' or 'T'
            num = int(m.group(2))        # 1..4

            # ---------- select container & PDO ----------
            pdo_container = self.node.rpdo if kind == 'R' else self.node.tpdo
            pdo = pdo_container[num]

            # ---------- clear previous mapping ----------
            pdo.clear()

            # ---------- map variables ----------
            for item in variables:
                if isinstance(item, (tuple, list)):
                    if len(item) == 2:
                        var_name, sub = item
                        pdo.add_variable(str(var_name), subindex=int(sub))
                    elif len(item) == 1:
                        pdo.add_variable(str(item[0]))
                    else:
                        raise ValueError(f"Bad variable spec: {item}")
                else:
                    pdo.add_variable(str(item))

            # ---------- compute COB-ID from standard bases ----------
            TPDO_BASES = {1: 0x180, 2: 0x280, 3: 0x380, 4: 0x480}
            RPDO_BASES = {1: 0x200, 2: 0x300, 3: 0x400, 4: 0x500}
            base = (RPDO_BASES if kind == 'R' else TPDO_BASES)[num]
            pdo.cob_id = int(base) + int(node_id)

            # ---------- transmission type & timers ----------
            pdo.trans_type = (254 if kind == 'R' else 255) if trans_type is None else int(trans_type)
            if event_timer is not None and hasattr(pdo, "event_timer"):
                pdo.event_timer = int(event_timer)

            # ---------- enable/disable ----------
            pdo.enabled = bool(enabled)

            # ---------- write to device ----------
            pdo.save()
            self.common.delay_SDO()

            # ---------- read-back verification (must already exist: self.verify_pdo) ----------
            ok = False
            try:
                ok = self.verify(node_id=node_id,
                                    pdo_name=pdo_name,
                                    variables=variables,
                                    trans_type=trans_type,
                                    event_timer=event_timer,
                                    enabled=enabled)
            except Exception as ve:
                self.log.print(f"VERIFY {pdo_name} exception", f"{ve}",
                            "❌", "PDO", "VERIFY")

            if not ok:
                self.log.print(f"{pdo_name} verification mismatch — config not accepted",
                            "❌", "PDO", "CONFIG")
                return None

            # ---------- success log ----------
            self.log.print(
                f"Configured {name} → vars={variables}, "
                f"COB-ID=0x{pdo.cob_id:03X}, TT={pdo.trans_type}, "
                f"ET={getattr(pdo, 'event_timer', 'n/a')}, enabled={pdo.enabled}",
                "PDO", "CONFIG", name, "✅"
            )
            return pdo

        except (canopen.SdoCommunicationError,
                canopen.SdoAbortedError,
                RuntimeError,
                KeyError,
                ValueError,
                AttributeError) as e:
            self.log.print(f"{pdo_name} config failed", f"{e}",
                        "❌", "PDO", str(pdo_name))
            return None
        except Exception as e:
            self.log.print(f"Unexpected error in {pdo_name}", f"{e}",
                        "❌", "PDO", str(pdo_name))
            return None

    def verify(self,
            node_id: int,
            pdo_name: str,                 # "RPDO1".."RPDO4" or "TPDO1".."TPDO4" (spaces ok)
            variables,                      # ["Target_position", ("Profile_velocity", 0)], etc.
            trans_type: int | None = None,  # default: RPDO=254, TPDO=255
            event_timer: int | None = None, # ms; used mainly for TPDOs
            enabled: bool = True) -> bool:
        """
        Read back an existing PDO and verify it matches the requested configuration.
        Does NOT change any device settings.
        """
        import re
        import canopen

        # pretty-printer for var tuples -> "(0x6077, 0, 16)" or "(0x6041, 0)"
        def _fmt_vars(seq):
            out = []
            for it in seq:
                try:
                    if len(it) == 3:
                        i, s, ln = it
                        out.append(f"(0x{i:04X}, {s}, {ln})")
                    elif len(it) == 2:
                        i, s = it
                        out.append(f"(0x{i:04X}, {s})")
                    else:
                        out.append(str(it))
                except Exception:
                    out.append(str(it))
            return "[" + ", ".join(out) + "]"

        try:
            if self.node is None:
                raise RuntimeError("Node not initialized. Call add_node_to_network() first.")

            # ---------- parse PDO name ----------
            name = re.sub(r"\s+", "", str(pdo_name).upper())
            m = re.fullmatch(r'(R|T)PDO([1-4])', name)
            if not m:
                raise ValueError(f"Invalid pdo_name '{pdo_name}'. Use 'RPDO1'..'RPDO4' or 'TPDO1'..'TPDO4'.")

            kind = m.group(1)              # 'R' or 'T'
            num = int(m.group(2))          # 1..4

            # ---------- pull live PDO config from device ----------
            if kind == 'R':
                self.node.rpdo.read()
                pdo_rb = self.node.rpdo[num]
            else:
                self.node.tpdo.read()
                pdo_rb = self.node.tpdo[num]

            actual_cob = int(getattr(pdo_rb, "cob_id"))
            actual_tt  = int(getattr(pdo_rb, "trans_type"))
            actual_et  = int(getattr(pdo_rb, "event_timer")) if hasattr(pdo_rb, "event_timer") else None
            actual_en  = bool(getattr(pdo_rb, "enabled"))
            actual_vars = [(mv.index, mv.subindex, mv.length) for mv in pdo_rb]

            # ---------- compute expected values (no device writes) ----------
            TPDO_BASES = {1: 0x180, 2: 0x280, 3: 0x380, 4: 0x480}
            RPDO_BASES = {1: 0x200, 2: 0x300, 3: 0x400, 4: 0x500}
            base = (RPDO_BASES if kind == 'R' else TPDO_BASES)[num]
            exp_cob = int(base) + int(node_id)
            exp_tt  = (254 if kind == 'R' else 255) if trans_type is None else int(trans_type)
            exp_et  = int(event_timer) if (event_timer is not None and kind == 'T') else None
            exp_en  = bool(enabled)

            # build expected mapping by resolving names via SDO/OD (no saves)
            exp_vars = []
            for item in variables:
                if isinstance(item, (tuple, list)):
                    if len(item) == 2:
                        var_name, sub = item
                        s = self.node.sdo[str(var_name)]
                        idx = int(s.index)
                        subidx = int(sub)
                    elif len(item) == 1:
                        var_name = item[0]
                        s = self.node.sdo[str(var_name)]
                        idx = int(s.index)
                        subidx = int(getattr(s, "subindex", 0))
                    else:
                        raise ValueError(f"Bad variable spec: {item}")
                else:
                    s = self.node.sdo[str(item)]
                    idx = int(s.index)
                    subidx = int(getattr(s, "subindex", 0))

                length_bits = None
                try:
                    length_bits = int(getattr(s, "bitsize", None) or getattr(s, "length", None))
                except Exception:
                    length_bits = None
                exp_vars.append((idx, subidx, length_bits))

            # If we couldn't resolve expected bit-lengths, compare index/sub only
            def _normalize(vars_list):
                return [(i, s) for (i, s, _lenbits) in vars_list]

            compare_by_len = all(l is not None for _i, _s, l in exp_vars)
            if compare_by_len:
                exp_vars_cmp = exp_vars
                act_vars_cmp = actual_vars
            else:
                exp_vars_cmp = _normalize(exp_vars)
                act_vars_cmp = [(i, s) for (i, s, _l) in actual_vars]

            # ---------- comparisons ----------
            ok_cob = (actual_cob == exp_cob)
            ok_tt  = (actual_tt  == exp_tt)
            ok_en  = (actual_en  == exp_en)
            ok_et  = True
            if exp_et is not None:
                ok_et = (actual_et is not None and actual_et == exp_et)
            ok_vars = (act_vars_cmp == exp_vars_cmp)

            ok_all = ok_cob and ok_tt and ok_et and ok_en and ok_vars

            # ---------- logging (hex indexes) ----------
            if ok_all:
                self.log.print(
                    (f"VERIFY {name} OK → "
                    f"COB-ID=0x{actual_cob:03X}, TT={actual_tt}, "
                    f"ET={actual_et if actual_et is not None else 'n/a'}, enabled={actual_en} | "
                    f"vars={_fmt_vars(actual_vars)}"),
                    "PDO", "VERIFY", name, "✅"
                )
            else:
                diffs = []
                if not ok_cob: diffs.append(f"COB-ID exp=0x{exp_cob:03X} got=0x{actual_cob:03X}")
                if not ok_tt:  diffs.append(f"TT exp={exp_tt} got={actual_tt}")
                if not ok_en:  diffs.append(f"EN exp={exp_en} got={actual_en}")
                if not ok_et:  diffs.append(f"ET exp={exp_et} got={actual_et}")
                if not ok_vars:
                    diffs.append(
                        "VARS exp=" + _fmt_vars(exp_vars_cmp) + " got=" + _fmt_vars(act_vars_cmp)
                        + ("" if compare_by_len else " (len ignored)")
                    )
                self.log.print(
                    f"VERIFY {name} failed: " + "; ".join(diffs),
                    "❌", "PDO", "VERIFY"
                )

            return ok_all

        except (canopen.SdoCommunicationError,
                canopen.SdoAbortedError,
                RuntimeError,
                KeyError,
                ValueError,
                AttributeError) as e:
            self.log.print(f"VERIFY {pdo_name}", f"{e}", "❌", "PDO", "VERIFY")
            return False
        except Exception as e:
            self.log.print(f"VERIFY {pdo_name} unexpected", f"{e}", "❌", "PDO", "VERIFY")
            return False