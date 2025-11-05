# Copyright 2024 Physical Intelligence
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0

import abc
from typing import Dict


class BasePolicy(abc.ABC):
    """Minimal interface for PI policies."""

    @abc.abstractmethod
    def infer(self, obs: Dict) -> Dict:
        """Infer actions from observations."""

    def reset(self) -> None:
        """Reset policy state."""
        return None

