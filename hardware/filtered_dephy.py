from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging import LOGGER


class FilteredDephyActuator(DephyActuator):
    """Dephy actuator with packet sanity checks and thermal jump rejection."""

    def __init__(
        self,
        *args,
        max_current_step_mA: float = 20000.0,
        max_case_temperature_step_C: float = 5.0,
        max_consecutive_rejections: int = 3,
        max_motor_current_mA: float = 60000.0,
        max_motor_voltage_mV: float = 10000.0,
        max_battery_current_mA: float = 5000.0,
        min_battery_voltage_mV: float = 25000.0,
        max_battery_voltage_mV: float = 45000.0,
        min_case_temperature_C: float = -20.0,
        max_case_temperature_C: float = 100.0,
        max_motor_acceleration_counts: float = 10000.0,
        max_motor_angle_step_counts: float = 50000.0,
        max_state_time_step_ms: float | None = None,
        max_consecutive_packet_rejections: int = 3,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)

        self.max_current_step_mA = max_current_step_mA
        self.max_case_temperature_step_C = max_case_temperature_step_C
        self.max_consecutive_rejections = max_consecutive_rejections
        self.max_motor_current_mA = max_motor_current_mA
        self.max_motor_voltage_mV = max_motor_voltage_mV
        self.max_battery_current_mA = max_battery_current_mA
        self.min_battery_voltage_mV = min_battery_voltage_mV
        self.max_battery_voltage_mV = max_battery_voltage_mV
        self.min_case_temperature_C = min_case_temperature_C
        self.max_case_temperature_C = max_case_temperature_C
        self.max_motor_acceleration_counts = max_motor_acceleration_counts
        self.max_motor_angle_step_counts = max_motor_angle_step_counts
        self.max_state_time_step_ms = (
            max_state_time_step_ms if max_state_time_step_ms is not None else max(50.0, 5.0 * 1000.0 / self.frequency)
        )
        self.max_consecutive_packet_rejections = max_consecutive_packet_rejections

        self.raw_thermal_current = 0.0
        self.filtered_thermal_current = 0.0
        self.raw_thermal_case_temperature = 0.0
        self.filtered_thermal_case_temperature = 0.0
        self.thermal_filter_rejections = 0
        self.packet_rejected = 0
        self.packet_rejection_count = 0
        self.packet_rejection_reason = ""
        self.packet_resync_count = 0
        self.raw_packet_state_time = 0.0
        self.raw_packet_mot_ang = 0.0
        self.raw_packet_mot_cur = 0.0
        self.raw_packet_mot_volt = 0.0
        self.raw_packet_batt_volt = 0.0
        self.raw_packet_temperature = 0.0

        self._current_rejection_count = 0
        self._case_temperature_rejection_count = 0
        self._has_filtered_current = False
        self._has_filtered_case_temperature = False
        self._last_valid_data = None
        self._soft_packet_rejection_count = 0
        self._soft_packet_rejection_category = ""

    def update(self) -> None:
        if (
            self._last_valid_data is None
            and self._data is not None
            and self._absolute_rejection_reason(self._data) is None
        ):
            self._last_valid_data = self._data.copy()

        candidate = self.read()
        self._store_raw_packet(candidate)

        rejection_reason, rejection_is_hard, rejection_category = self._packet_rejection(candidate)
        if rejection_reason is None:
            self._accept_packet(candidate)
        elif not rejection_is_hard and self._should_resync_soft_rejection(rejection_category):
            self._resync_packet(candidate, rejection_reason)
        else:
            self._reject_packet(candidate, rejection_reason, rejection_is_hard, rejection_category)

        self.raw_thermal_current = self.motor_current
        self.raw_thermal_case_temperature = self.case_temperature

        self.filtered_thermal_current = self._filter_jump(
            name="motor_current",
            raw_value=self.raw_thermal_current,
            previous_value=self.filtered_thermal_current,
            max_step=self.max_current_step_mA,
            rejection_count_attr="_current_rejection_count",
            has_previous=self._has_filtered_current,
        )
        self._has_filtered_current = True

        self.filtered_thermal_case_temperature = self._filter_jump(
            name="case_temperature",
            raw_value=self.raw_thermal_case_temperature,
            previous_value=self.filtered_thermal_case_temperature,
            max_step=self.max_case_temperature_step_C,
            rejection_count_attr="_case_temperature_rejection_count",
            has_previous=self._has_filtered_case_temperature,
        )
        self._has_filtered_case_temperature = True

        self._thermal_scale = self._thermal_model.update(
            dt=1 / self.frequency,
            motor_current=self.filtered_thermal_current,
            case_temperature=self.filtered_thermal_case_temperature,
        )

        self._check_i2t_fault()

    def _store_raw_packet(self, data) -> None:
        self.raw_packet_state_time = float(data.get("state_time", 0.0))
        self.raw_packet_mot_ang = float(data.get("mot_ang", 0.0))
        self.raw_packet_mot_cur = float(data.get("mot_cur", 0.0))
        self.raw_packet_mot_volt = float(data.get("mot_volt", 0.0))
        self.raw_packet_batt_volt = float(data.get("batt_volt", 0.0))
        self.raw_packet_temperature = float(data.get("temperature", 0.0))

    def _accept_packet(self, data) -> None:
        self._data = data.copy()
        self._last_valid_data = data.copy()
        self.packet_rejected = 0
        self.packet_rejection_reason = ""
        self._soft_packet_rejection_count = 0
        self._soft_packet_rejection_category = ""

    def _reject_packet(self, data, reason: str, is_hard: bool, category: str) -> None:
        self.packet_rejected = 1
        self.packet_rejection_count += 1
        self.packet_rejection_reason = reason
        if is_hard:
            self._soft_packet_rejection_count = 0
            self._soft_packet_rejection_category = ""

        if self._last_valid_data is None:
            self._data = data.copy()
            self._last_valid_data = data.copy()
            LOGGER.warning(
                f"[{self.tag.upper()}] Accepted first packet despite rejection reason '{reason}' "
                "because no previous valid packet exists."
            )
            return

        self._data = self._last_valid_data.copy()
        if self.packet_rejection_count == 1 or self.packet_rejection_count % 10 == 0:
            LOGGER.warning(
                f"[{self.tag.upper()}] Rejected {'hard' if is_hard else 'soft'} actuator packet: {reason}. "
                f"count={self.packet_rejection_count}, "
                f"raw_state_time={data.get('state_time')}, raw_mot_ang={data.get('mot_ang')}, "
                f"raw_mot_cur={data.get('mot_cur')}, raw_mot_volt={data.get('mot_volt')}, "
                f"raw_batt_volt={data.get('batt_volt')}, raw_temperature={data.get('temperature')}."
            )

    def _resync_packet(self, data, reason: str) -> None:
        self.packet_resync_count += 1
        self._data = data.copy()
        self._last_valid_data = data.copy()
        self.packet_rejected = 0
        self.packet_rejection_reason = f"resynced after {reason}"
        self._soft_packet_rejection_count = 0
        self._soft_packet_rejection_category = ""
        LOGGER.warning(
            f"[{self.tag.upper()}] Resynced actuator packet after persistent soft rejection: {reason}. "
            f"resync_count={self.packet_resync_count}, raw_state_time={data.get('state_time')}, "
            f"raw_mot_ang={data.get('mot_ang')}."
        )

    def _should_resync_soft_rejection(self, category: str) -> bool:
        if category == self._soft_packet_rejection_category:
            self._soft_packet_rejection_count += 1
        else:
            self._soft_packet_rejection_category = category
            self._soft_packet_rejection_count = 1

        return self._soft_packet_rejection_count >= self.max_consecutive_packet_rejections

    def _packet_rejection(self, data) -> tuple[str | None, bool, str]:
        absolute_reason = self._absolute_rejection_reason(data)
        if absolute_reason is not None:
            return absolute_reason, True, "absolute"

        if self._last_valid_data is None:
            return None, False, ""

        state_time = data.get("state_time")
        last_state_time = self._last_valid_data.get("state_time")
        if state_time is not None and last_state_time is not None:
            state_time_step = state_time - last_state_time
            if state_time_step < 0:
                return f"state_time went backward by {abs(state_time_step)} ms", True, "state_time_backward"
            if state_time_step > self.max_state_time_step_ms:
                return f"state_time jumped by {state_time_step} ms", False, "state_time_forward"

        mot_ang = data.get("mot_ang")
        last_mot_ang = self._last_valid_data.get("mot_ang")
        if mot_ang is not None and last_mot_ang is not None:
            motor_angle_step = abs(mot_ang - last_mot_ang)
            if motor_angle_step > self.max_motor_angle_step_counts:
                return f"mot_ang jumped by {motor_angle_step} counts", False, "mot_ang"

        return None, False, ""

    def _absolute_rejection_reason(self, data) -> str | None:
        checks = [
            ("mot_cur", -self.max_motor_current_mA, self.max_motor_current_mA),
            ("mot_volt", -self.max_motor_voltage_mV, self.max_motor_voltage_mV),
            ("batt_volt", self.min_battery_voltage_mV, self.max_battery_voltage_mV),
            ("batt_curr", -self.max_battery_current_mA, self.max_battery_current_mA),
            ("temperature", self.min_case_temperature_C, self.max_case_temperature_C),
            ("mot_acc", -self.max_motor_acceleration_counts, self.max_motor_acceleration_counts),
            ("status_mn", 0, 255),
            ("status_ex", 0, 255),
            ("status_re", 0, 255),
        ]

        for field, min_value, max_value in checks:
            value = data.get(field)
            if value is None:
                continue
            if value < min_value or value > max_value:
                return f"{field}={value} outside [{min_value}, {max_value}]"

        return None

    def _filter_jump(
        self,
        name: str,
        raw_value: float,
        previous_value: float,
        max_step: float,
        rejection_count_attr: str,
        has_previous: bool,
    ) -> float:
        if not has_previous:
            setattr(self, rejection_count_attr, 0)
            return raw_value

        delta = abs(raw_value - previous_value)
        if delta <= max_step:
            setattr(self, rejection_count_attr, 0)
            return raw_value

        rejection_count = getattr(self, rejection_count_attr) + 1
        setattr(self, rejection_count_attr, rejection_count)

        if rejection_count <= self.max_consecutive_rejections:
            self.thermal_filter_rejections += 1
            LOGGER.warning(
                f"[{self.tag.upper()}] Rejected thermal {name} jump: "
                f"raw={raw_value:.3f}, filtered={previous_value:.3f}, "
                f"delta={delta:.3f}, limit={max_step:.3f}, "
                f"count={rejection_count}/{self.max_consecutive_rejections}."
            )
            return previous_value

        LOGGER.warning(
            f"[{self.tag.upper()}] Accepting persistent thermal {name} jump after "
            f"{rejection_count} samples: raw={raw_value:.3f}, "
            f"previous={previous_value:.3f}, delta={delta:.3f}."
        )
        setattr(self, rejection_count_attr, 0)
        return raw_value
