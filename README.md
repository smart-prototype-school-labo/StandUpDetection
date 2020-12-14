# Stand-up-detection-device
立ち上がり検知

ハッカソンで作ったやつです。
介護施設向けの簡易な立ち上がり検知システムです。

M5Stickで角度検知
→ 50°以上だったらFirebase RealtimeDBに通知
→ RealTimeDBを通知しているFlutterAppで音が鳴る
