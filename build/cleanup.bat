echo Let's clean them up.

echo Cleaning vs2005\...
del vs2005\*.ncb
del vs2005\*.user
del vs2005\*.suo /a:h
rmdir vs2005\obj /s /q

echo Cleaning vs2008\...
del vs2008\*.ncb
del vs2008\*.user
del vs2008\*.suo /a:h
rmdir vs2008\obj /s /q

echo Cleaning vs2010\...
del vs2010\*.sdf
del vs2010\*.user
del vs2010\*.suo /a:h
rmdir vs2010\ipch /s /q
rmdir vs2010\obj /s /q

echo Cleaning vs2012\...
del vs2012\*.sdf
del vs2012\*.user
del vs2012\*.suo /a:h
rmdir vs2012\ipch /s /q
rmdir vs2012\obj /s /q

echo Cleaning vs2013\...
del vs2013\*.sdf
del vs2013\*.user
del vs2013\*.suo /a:h
rmdir vs2013\ipch /s /q
rmdir vs2013\obj /s /q

echo Cleaning vs2015\...
del vs2015\*.sdf
del vs2015\*.user
del vs2015\*.suo /a:h
rmdir vs2015\.vs /s /q
rmdir vs2015\obj /s /q

echo Finished. Yeah !
