"""Localize skill.

Search landmark to localize.
"""

from ..headskills.ScanField import ScanField
from ...misc.bt import Action, parallel

sc = ScanField()


class _Localize(Action):
    def tick(self):
        self.enable_localization()
        self.crouch()

        sc.tick()
        return self.success()


Localize = parallel(_Localize)
