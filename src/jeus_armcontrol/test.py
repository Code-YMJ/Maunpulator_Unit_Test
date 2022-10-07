from jeus_log import *

log = jeus_log('C:\Project', 'test')
log.Warning('Warning')
log.Error('Error')
log.Debug('Debug')
log.Info('Info')
log.Critical('Critical')
a=2
log.Critical('Joint_%02d'%a)
log.Critical('%02d'%a)