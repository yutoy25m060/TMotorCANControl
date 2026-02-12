# TMotorCANControl
CubeMars社のAKシリーズT-MotorアクチュエータをCANバス経由で制御するためのPython APIです。
このプロジェクトは、Raspberry PiのCANハットまたはシリアルバスを使用してAK80-9アクチュエータを制御することに主眼を置いていますが、
他のCAN/シリアルインターフェースにも容易に適応可能です。APIファイルは、このリポジトリの `src/TMotorCANControl` フォルダにあります。
メインのインターフェースは、MITモード用の `TMotorManager_mit_can.py`、サーボモード（CAN経由）用の `TMotorManager_servo_can.py`、そしてサーボモード（シリアル経由）用の `TMotorManager_servo_serial` です。
サンプルスクリプトは `demos` フォルダにあります。

モーターのセットアップに関するビデオ手順については、
[Yoyo Liu氏のYouTubeチャンネル](https://www.youtube.com/watch?v=iJBJhivWqxE)にあるCubeMarsのチュートリアルをご覧ください。
モーターのセットアップとPiの設定に関する書面の手順については、
Open Source Legウェブサイトの[こちらの説明](https://opensourceleg.com/cubemars-tmotor-control-method/)をご覧ください。

## APIの使用方法
いくつかのコード例については、このリポジトリの `demos` フォルダを参照してください。
完全なAPIドキュメントについては、[ReadTheDocsの私たちのページ](https://tmotorcancontrol.readthedocs.io/en/latest/index.html)をご覧ください。
これらの例では、制御ループに[NeuroLocoMiddlewareライブラリ](https://pypi.org/project/NeuroLocoMiddleware/)の `soft_real_timeloop` クラスを使用していますが、このライブラリに依存せずに使用することも可能です。

`TMotorManager_mit_can`、`TMotorManager_servo_can`、および `TMotorManager_servo_serial` クラスは、`TMotorCANControl` パッケージの `TMotorManager` モジュールにあります。意図された使用法は、`TMotorManager_mit_can`、`TMotorManager_servo_can`、または `TMotorManager_servo_serial` オブジェクトをブロック内で宣言し、そのブロック内でコントローラーを記述することです。これにより、使用中はモーターの電源がオンになり、エラーがスローされたりプログラムが終了したりした場合には電源がオフになることが保証されます。

CANバス経由でMIT制御用にセットアップされたモーターを制御するには、
以下のように、CAN ID 3のAK80-9用に[TMotorManager_mit_canオブジェクト](https://tmotorcancontrol.readthedocs.io/en/latest/TMotorCANControl.html#TMotorCANControl.mit_can.TMotorManager_mit_can)を作成します。
```python
from TMotorCANControl.TMotorManager_mit_can import TMotorManager_mit_can
with TMotorManager_mit_can(motor_type='AK80-9', motor_ID=3) as dev:
    dev.update()
```

CANバス経由でサーボ制御用にセットアップされたモーターを制御するには、
以下のように、CAN ID 3のAK80-9用に[TMotorManager_servo_canオブジェクト](https://tmotorcancontrol.readthedocs.io/en/latest/TMotorCANControl.html#TMotorCANControl.servo_can.TMotorManager_servo_can)を作成します。
```python
from TMotorCANControl.TMotorManager_servo_can import TMotorManager_servo_can
with TMotorManager_servo_can(motor_type='AK80-9', motor_ID=3) as dev:
    dev.update()
```

シリアルバス経由でサーボ制御用にセットアップされたモーターを制御するには、
以下のように、USBシリアルポート '/dev/ttyUSB0'、ボーレート 921600（デフォルト）のAK80-9用に[TMotorManager_servo_serialオブジェクト](https://tmotorcancontrol.readthedocs.io/en/latest/TMotorCANControl.html#TMotorCANControl.servo_serial.TMotorManager_servo_serial)を作成します。
```python
from TMotorCANControl.TMotorManager_servo_serial import TMotorManager_servo_serial
with TMotorManager_servo_serial(port='/dev/ttyUSB0', baud=921600, motor_params=Servo_Params_Serial['AK80-9']) as dev:
    dev.update()
```

モーターは、使用している通信設定に応じて、以下の表に示すさまざまなモードで制御できます。

| 制御モード                               | MIT CAN | Servo CAN | Servo Serial |
| ------------------------------------------ | ------- | --------- | ------------ |
| 電流                                       | はい    | はい      | はい         |
| 速度                                       | はい    | はい      | はい         |
| 位置                                       | はい    | はい      | はい         |
| デューティサイクル                         | いいえ  | はい      | はい         |
| フィードフォワード電流付きインピーダンス   | はい    | いいえ    | いいえ       |
| 速度/加速度制限付き位置                    | いいえ  | はい      | はい         |

一度モードに入ると、TMotorManagerの内部コマンドを設定し、`update()`メソッドを呼び出してコマンドを送信することで、これらのいずれかのモードでモーターを制御できます。内部コマンドの値は、使用している通信プロトコルで利用可能な場合、以下のメソッドで設定できます。

- `set_output_angle_radians(pos)`: 位置指令を "pos" ラジアンに設定します。
- `set_motor_current_qaxis_amps(current)`: 電流指令を "current" アンペアに設定します。
- `set_duty_cycle_percent(duty)`: (サーボモードのみ、CANまたはシリアル) デューティサイクルを指定されたパーセンテージ比率（0から1の間）に設定します。
- `set_output_torque_newton_meters(torque)`: 指定されたトルクに基づいて電流指令を設定します。
- `set_output_velocity_radians_per_second(vel)`: 速度指令を "vel" rad/sに設定します。
- `set_motor_torque_newton_meters(torque)`: モーター側トルクを制御するために、ギア比で調整された指定トルクに基づいてトルク指令を設定します。
- `set_motor_angle_radians(pos)`: モーター側位置を制御するために、ギア比で調整された指定位置に基づいて位置指令を設定します。
- `set_motor_velocity_radians_per_second(vel)`: モーター側速度を制御するために、ギア比で調整された指定速度に基づいて速度指令を設定します。

さらに、モーターの状態は以下のメソッドでアクセスできます。状態は`update()`メソッドが呼ばれるたびに更新され、メソッド名は自己説明的です。
- `get_current_qaxis_amps()`
- `get_output_angle_radians()`
- `get_output_velocity_radians_per_second()`
- `get_output_acceleration_radians_per_second_squared()`
- `get_output_torque_newton_meters()`
- `get_motor_angle_radians()`
- `get_motor_velocity_radians_per_second()`
- `get_motor_acceleration_radians_per_second_squared()`
- `get_motor_torque_newton_meters()`
- `get_motor_error_code()`

以下はサーボモード（シリアルバス経由）でのみ利用可能です：
- `get_current_daxis_amps()`
- `get_current_bus_amps()`
- `get_voltage_qaxis_volts()`
- `get_voltage_daxis_volts()`
- `get_voltage_bus_volts()`

上記のゲッターとセッターは、使いやすさのためにプロパティとしても結合されています。

- `current_qaxis`: q軸電流（アンペア）
- `position`: 出力角度（ラジアン、ギアボックス後）
- `velocity`: 出力速度（rad/s、ギアボックス後）
- `acceleration`: 出力加速度（rad/s/s、ギアボックス後）
- `torque`: 出力トルク（Nm、ギアボックス後）
- `position_motorside`: モーター側角度（ラジアン、ギアボックス前）
- `velocity_motorside`: モーター側速度（rad/s、ギアボックス前）
- `acceleration_motorside`: モーター側加速度（rad/s/s、ギアボックス前）
- `torque_motorside`: モーター側トルク（Nm、ギアボックス前）

そして、サーボモード（シリアルバス経由）でのみ：
- `current_daxis`: d軸電流（アンペア）
- `current_bus`: 入力電流（アンペア）
- `voltage_qaxis`: q軸電圧（ボルト）
- `voltage_daxis`: d軸電圧（ボルト）
- `voltage_bus`: 入力電圧（ボルト）

もう一つの特筆すべき関数は `zero_position()` 関数です。これはモーターに現在の角度をゼロにするコマンドを送ります。この関数は、モーターがゼロ調整を行う間（スケールをゼロにするのと同様に、良い測定値を得るためにいくつかの点を記録するようです）、約半秒間モーターの制御をオフにします。したがって、タイムリーな通信が重要な場合は、このメソッドを呼び出した後、少なくとも500ミリ秒待機する必要があります。

以下の例では、CAN ID 3のAK80-9モーター用のTMotorManagerをインスタンス化し、"log.csv"という名前のCSVファイルに、上記の全ログ変数を記録します。その後、モーターの位置をゼロにし、モーターがゼロ調整を完了するのに十分な時間待ちます。最後に、ゲイン 10Nm/rad と 0.5Nm/(rad/s) でインピーダンス制御モードに入ります。そして、プログラムが終了するまでモーターの位置を3.14ラジアンに設定します。

```python
with TMotorManager_mit_can(motor_type='AK80-9', motor_ID=3) as dev:
    dev.set_zero_position()
    time.sleep(1.5)
    dev.set_impedance_gains_real_unit(K=10,B=0.5)
    loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)

    for t in loop:
        dev.update()
        dev.position = 3.14
```

その他の例については、`demos` フォルダを参照してください。CubeMarsモーターの制御を楽しんでください！

## その他のリソース
1. [OSLウェブサイトのセットアップ手順](https://opensourceleg.com/cubemars-tmotor-control-method/)

2. [APIドキュメント](https://tmotorcancontrol.readthedocs.io/en/latest/index.html)

3. [AKシリーズモーターマニュアル](https://store.cubemars.com/images/file/20211201/1638329381542610.pdf)
AKシリーズT-Motorsのドキュメント。CANプロトコルとR-Linkの使用方法が含まれています。

4. [PiCAN 2 CAN Bus Hat](https://copperhilltech.com/pican-2-can-bus-interface-for-raspberry-pi/) 
CopperHill Raspberry Pi CANハットのドキュメント。

5. [RLink YouTubeビデオ](https://www.youtube.com/channel/UCs-rBZ4uKBpOT9vokLZPhog/featured)
Yoyo氏のYouTubeチャンネルには、RLinkソフトウェアの使用方法に関するチュートリアルがあります。

6. [Mini-Cheetah-TMotor-Python-Can](https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-python-can)
これは、MITモードでこれらのモーターを制御するための、より低レベルな別のライブラリです。

この作業はMitry Anderson氏とVamsi Peddinti氏によって行われました。
