著者：根岸孝次

めちゃめちゃ重要なIMU、ジャイロセンサについて解説します。

## 概要
### IMUとは
IMU（Inertial Measurement Unit、慣性計測装置）は、物体の加速度や角速度を測定し、位置や姿勢を推定するためのデバイスです。IMUは通常、3軸の加速度計と3軸のジャイロスコープを備え、3次元空間での動きを捉えます。場合によっては、3軸の磁力計が追加され、方位角も計測できます。IMUは、ロボット、自動車、ドローンなどで位置制御やナビゲーションに利用されます。データはフィルタ処理を通じてノイズを抑え、より正確な姿勢推定を可能にします。

ロボカップでは、ロボットがどの向きを向いているかを把握するために利用します。コートに垂直な軸周りの回転量を求めます。

## ジャイロセンサとは
ジャイロセンサ（ジャイロスコープ）は、物体の回転や角速度を検出するセンサで、角速度の変化を測定することで物体の姿勢や方向を把握します。3軸のジャイロセンサは、X、Y、Zの各軸に対する回転を測定でき、3次元空間での回転動作を解析するために使用されます。この技術は、ドローンやスマートフォン、車両の安定化や姿勢制御に広く利用されています。ジャイロセンサの出力は、フィルタリングされることで精度が向上し、IMUと組み合わせてさらに安定した姿勢推定が可能になります。

他にも加速度計、磁力計がありますが、これらを組み合わせて物体の姿勢を推定するためのデバイスがIMUとなります。
$$
加速度計 \subset IMU, \quad ジャイロスコープ \subset IMU, \quad 磁力計 \subset IMU
$$

ロボカップで機体の角度を推定するために最も活躍するものがジャイロスコープです。まずはジャイロスコープについて軽く解説します。

ジャイロスコープは3つの軸（$x,y,z$軸）周りの角速度（$rad/s$）を計測するデバイスです。マイコンでI2Cなどを利用してそれらの値を取得します。我々がほしいのは角度（$rad$）ですので、角速度を積分してあげればよいわけです。それぞれの軸周りの回転量を$\phi, \theta, \psi$とすると、それぞれの軸周りの角速度は$\frac{d\phi}{dt}, \frac{d\theta}{dt}, \frac{d\psi}{dt}$となります。したがって、
$$
\phi (t) = \int_{0}^{t} \frac{d\phi}{dt}dt + \phi_0, \quad \theta (t) = \int_{0}^{t} \frac{d\theta}{dt}dt + \theta_0, \quad \psi (t) = \int_{0}^{t} \frac{d\psi}{dt}dt + \psi_0
$$
となります。ロボカップでは、コートに垂直な軸周りの回転量だけわかれば十分です。ジャイロセンサの配置を工夫して、$x,y,z$軸のいずれかがコートと垂直になるようにしてあげれば、計算が単純になります。（ジャイロセンサの基板やデータシートに大体軸の正に方向は記してあります。）この資料では、$z$軸がコートと垂直であると仮定して話を進めていきます。


### 実際に積分をする
積分をすれば良いと言いましたが、数学でやるような完璧な積分は残念ながら不可能です。そもそも、ジャイロセンサが計測してくれる角速度が連続な値ではなく周期的に計測して離散的な値だからです。また、マイコンも処理速度に限界があるので連続に処理を行うことができません。

積分の基礎に戻って考えます。角速度が$\dot{\psi}(t)$で短い時間$\Delta t$の間の角度変化は$\Delta \psi(t) = \dot{\psi}\Delta t$と表せますね。したがって、
$$
\psi(t+\Delta t) = \psi(t) + \Delta \psi(t) = \psi(t) + \dot{\psi} \Delta t
$$となります。$\Delta t$を十分小さくして行けば、$\psi (t)$はある程度正確な値になるはずです。実際の制御ではこのようにして角度を求めていきます。ちなみに、この考え方より簡単にもう少しだけ精度が**上がるかもしれない**方法もあります。先程の方法では細かい長方形で面積を近似する方法でしたが、台形で近似することも可能です。

$\Delta t$の調べ方ですが、Arduinoではmills関数とmicros関数が存在します。この関数は、Arduinoが起動してから何ミリ、マイクロ秒経過したかを返してくれる関数です。（※1,2）積分をロープ実行すると思うので、前回積分を行ったときの時刻と次に積分を行う時刻の差から$\Delta t$が求まります。それぞれの関数から求まる$\Delta t$の単位は$ms, \micro s$です。積分をする際には、単位を$s$に変換することを忘れないでください。
$$
\Delta t = t_{now} - t_{past}
$$
※1: 起動してからの時間というのは正確には誤りです。時間を格納する変数にも格納できる数の最大値があります。それを越えてしまうとオーバーフローという現象が起こり、0から再度スタートすることになります。今回の使い方だと$\Delta t$がわかればよいので時刻が正確でなくても問題ではありません。
※2: mills関数とmicros関数のどちらを利用するかはそれぞれを試してデバッグをしながら決めていけば良いと思います。積分以外に行っている処理やマイコンの処理能力にかなり依存するので注意が必要です。また、STM32マイコンなどではタイマー割り込みの設定が容易なので$\Delta t$を固定して積分を行うことも可能です。

### 補正をする
残念ながら積分して得られる角度はかなり信頼できないことが多いです。主に2つの要因があります。
- ドリフト
	- 静止しているときの角速度が$0$でないことが原因
	- 機体が静止していても角度が勝手に増えていく（減っていく）
		- 実際に確認してみることが重要
	- 機体を数秒間静止させた時の角速度の平均を算出し、それをオフセットとして、センサから取得した角速度からオフセットを引いた値を真の角速度として利用する。$$ \psi = \psi_{raw} - \psi_{offset} $$
- スケールファクター誤差
	- センサから取得した角速度$\psi_{raw}$が真の値$\psi_{true}$としたときに、定常的に$r$倍となっている現象。$$ r = \frac{\psi_{raw}}{\psi_{true}}$$
	- 機体をピッタリ10回転程度させたときとの$\psi$の変化量$\Delta \psi$を$20\pi$で割ることで$r$を求める。$$ r = \frac{\Delta \psi}{20\pi} $$
	- ドリフトの補正をして得られた角速度を$r$で割って利用する。

最低限ドリフトの補正はやる必要があります。スケールファクター誤差は必要に応じてやってください。ロボットの電源を付けたら毎回これらの補正をやるのが理想ですが、手間なので定期的にやると良いと思います。

## 加速度センサについて
名前の通り3つの軸方向の加速度を計測するセンサです。実は、加速度からも機体の角度を算出できます。（yaw軸まわりの角度は求められないと何処かで聞きました。真偽不明。）地球上では常に重力加速度$g$が地面に垂直で下向きにかかっています。機体の角度に応じてこれを$a_x, a_y, a_z$に分解できます。この事実を利用して角度を求めるようです。自分で利用したことがないので詳しい計算式などについては[ここ](https://garchiving.com/angle-from-acceleration/)などを参考にしてみるとよいのではないでしょうか？
```cardlink
url: https://garchiving.com/angle-from-acceleration/
title: "加速度センサーから角度の変換計算方法"
description: "加速度センサーの取得値（重力ベクトル）から角度の計算方法を簡単に解説します。最後はサンプルプログラムから姿勢角度算出の動作確認をしてみます。"
host: garchiving.com
favicon: https://garchiving.com/wordpress/wp-content/uploads/2023/02/cropped-230218_00_imo_logo_512-32x32.jpg
image: https://garchiving.com/wordpress/wp-content/uploads/2016/04/00_eye-catchi_compass_00.jpg
```


## 磁力計について
磁力計も名前の通り磁力を計測するセンサです。地磁気センサとも呼びます。3つの軸それぞれの方向の磁力の強さと向きがわかります。（S極とN極どちらが正であるかなどはデータシートを参考にしてください。）実は、これを利用しても機体の姿勢を推定することができます。びっくり。

地球自体が1つの磁石であることは皆さんご存知のはずです。北極がS極、南極がN極ですね。これは地球上どこに行っても同じです。これを利用して姿勢を推定します。詳しい計算方法などは忘れたので調べてください。[ここ](https://qiita.com/take4eng/items/8afd467e82e2c93f19e4)などで解説されています。
```cardlink
url: https://qiita.com/take4eng/items/8afd467e82e2c93f19e4
title: "地磁気センサでオイラー角の姿勢推定 - Qiita"
description: "地磁気センサで3次元の姿勢推定地磁気センサから方位のデータを得られる。方位ベクトルの変位から、3次元空間での姿勢を計算できる。本記事では、地磁気センサからオイラー角での姿勢推定を行うための内容…"
host: qiita.com
favicon: https://cdn.qiita.com/assets/favicons/public/production-c620d3e403342b1022967ba5e3db1aaa.ico
image: https://qiita-user-contents.imgix.net/https%3A%2F%2Fqiita-user-contents.imgix.net%2Fhttps%253A%252F%252Fcdn.qiita.com%252Fassets%252Fpublic%252Farticle-ogp-background-afbab5eb44e0b055cce1258705637a91.png%3Fixlib%3Drb-4.0.0%26w%3D1200%26blend64%3DaHR0cHM6Ly9xaWl0YS11c2VyLXByb2ZpbGUtaW1hZ2VzLmltZ2l4Lm5ldC9odHRwcyUzQSUyRiUyRnBicy50d2ltZy5jb20lMkZwcm9maWxlX2ltYWdlcyUyRjEyMjE0MTAwMzgyNzI1NTcwNTglMkZaQ29sWmRad19iaWdnZXIuanBnP2l4bGliPXJiLTQuMC4wJmFyPTElM0ExJmZpdD1jcm9wJm1hc2s9ZWxsaXBzZSZmbT1wbmczMiZzPWI3ZDFkYWJhNDNjMWZjZmQxNDQzZjkwMjllODRmZGEz%26blend-x%3D120%26blend-y%3D467%26blend-w%3D82%26blend-h%3D82%26blend-mode%3Dnormal%26s%3Dea74250e37c244dbfd091c0711955fdc?ixlib=rb-4.0.0&w=1200&fm=jpg&mark64=aHR0cHM6Ly9xaWl0YS11c2VyLWNvbnRlbnRzLmltZ2l4Lm5ldC9-dGV4dD9peGxpYj1yYi00LjAuMCZ3PTk2MCZoPTMyNCZ0eHQ9JUU1JTlDJUIwJUU3JUEzJTgxJUU2JUIwJTk3JUUzJTgyJUJCJUUzJTgzJUIzJUUzJTgyJUI1JUUzJTgxJUE3JUUzJTgyJUFBJUUzJTgyJUE0JUUzJTgzJUE5JUUzJTgzJUJDJUU4JUE3JTkyJUUzJTgxJUFFJUU1JUE3JUJGJUU1JThCJUEyJUU2JThFJUE4JUU1JUFFJTlBJnR4dC1hbGlnbj1sZWZ0JTJDdG9wJnR4dC1jb2xvcj0lMjMxRTIxMjEmdHh0LWZvbnQ9SGlyYWdpbm8lMjBTYW5zJTIwVzYmdHh0LXNpemU9NTYmdHh0LXBhZD0wJnM9MmExZDUyOGQ3MjQwZjU5MDRjMjJjMjJlNmZmYWFhZDg&mark-x=120&mark-y=112&blend64=aHR0cHM6Ly9xaWl0YS11c2VyLWNvbnRlbnRzLmltZ2l4Lm5ldC9-dGV4dD9peGxpYj1yYi00LjAuMCZ3PTgzOCZoPTU4JnR4dD0lNDB0YWtlNGVuZyZ0eHQtY29sb3I9JTIzMUUyMTIxJnR4dC1mb250PUhpcmFnaW5vJTIwU2FucyUyMFc2JnR4dC1zaXplPTM2JnR4dC1wYWQ9MCZzPTFkYTAxYWE1ZWIzOTBlODA5MmNkNTVjOWYwOTRiNTUz&blend-x=242&blend-y=480&blend-w=838&blend-h=46&blend-fit=crop&blend-crop=left%2Cbottom&blend-mode=normal&s=0c70be66153bb379e87447cc19ea91d8
```

ただ、地磁気を利用した姿勢の推定には大きな弱点があります。この方法では、他の磁場に大きな影響を受けてしまいます。地磁気が微弱なせいですね。ロボカップジュニアサッカーではモーターやキッカー、ドリブラーのアクチュエータが強力な磁場を発生させます。機体サイズが小さいロボカップジュニアではこれらの磁場の影響をもろに受けてしまいます。残念。アクチュエータの動作に応じて磁場が目まぐるしく変化するので補正することも困難だと思います。（やったことない）1度、地磁気を利用して姿勢制御をやってみましたが話にならなかったです。機体の上部、ハンドルに近いところにセンサを取り付けることで磁場の影響を抑えているチームもあるみたいです。個人的には、ハンドルにセンサをつけることは許せないですけど。経験を積むために一度試してみるのは良いと思いますが、実際に使用するには難しいと思います。

### センサフージョンについて
ジャイロスコープ、加速度計、磁力計のそれぞれの特徴について軽く解説しました。ロボカップジュニアサッカーではジャイロスコープが有力です。しかし、ジャイロスコープ単体では蓄積していく誤差を補正することができません。そこで、2つ（もしくは3つ）のセンサからの値をあわせて利用する技術があります。
## DMPについて
