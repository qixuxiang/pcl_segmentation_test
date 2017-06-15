"""Penalised state."""

from ..skills.ReEntry import ReEntry
from misc.bt import Action, parallel, sequence


class _Penalised(Action):
    def tick(self):
        self.stand()
        # print 'Penalised behaviour ticking'
        self.world.enable_reentry = True
        return self.success()


Penalised = sequence(_Penalised)
