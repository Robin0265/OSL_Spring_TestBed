from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging import LOGGER


class FilteredDephyActuator(DephyActuator):
    """Dephy actuator with one-sample thermal input jump rejection."""

    def __init__(
        self,
        *args,
        max_current_step_mA: float = 20000.0,
        max_case_temperature_step_C: float = 5.0,
        max_consecutive_rejections: int = 3,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)

        self.max_current_step_mA = max_current_step_mA
        self.max_case_temperature_step_C = max_case_temperature_step_C
        self.max_consecutive_rejections = max_consecutive_rejections

        self.raw_thermal_current = 0.0
        self.filtered_thermal_current = 0.0
        self.raw_thermal_case_temperature = 0.0
        self.filtered_thermal_case_temperature = 0.0
        self.thermal_filter_rejections = 0

        self._current_rejection_count = 0
        self._case_temperature_rejection_count = 0
        self._has_filtered_current = False
        self._has_filtered_case_temperature = False

    def update(self) -> None:
        self._data = self.read()

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
