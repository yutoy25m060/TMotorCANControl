# CubeMars (TMotor) Control Method

出典: Open-Source Leg (OSL) プロジェクト ドキュメント（最終更新: 2022年12月16日）

本チュートリアルは、Dephyアクチュエータの代替として、CubeMars（TMotor）製のAK-seriesアクチュエータを使用する方法を解説する。これらのアクチュエータには低レベルのモーター制御を担うドライバチップが搭載されており、CANバスまたはシリアルポート経由でオープンソースのPythonライブラリ「[TMotorCANControl](https://github.com)」を用いて制御できる。本ライブラリはAK80-9で動作確認済みであり、AK-series全般で動作する見込みである（名称に反しシリアルモードにも対応）。

Raspberry Piには、モーターに加えてIMU、ロードセルアンプ、外部エンコーダなどの周辺機器も接続可能で、AK80-9モーター制御と同時にセンサーデータを取得できる（`opensourceleg` Pythonライブラリ、開発中）。

以下の制御例は、Dephyチュートリアルと同様に、より高レベルな制御戦略を実現するための基本的な構成要素（ビルディングブロック）として位置づけられる。

---

## 参考リンク

- **TMotorCANControl GitHub Repository** — パッケージのソースコード。デモスクリプトも同梱。
- **TMotorCANControl API documentation (ReadTheDocs)** — 各クラス・メソッド・変数の詳細ドキュメント。特に重要なのは `TMotorManager_mit_can`、`TMotorManager_servo_can`、`TMotorManager_servo_serial` クラス。
- **TMotorCANControl PyPI (pip) Page** — pipでインストール可能な最新版。
- **CubeMars AK80-seriesモーターマニュアル** — CubeMars GUIの使用法と各モードの通信プロトコルを解説。

---

## OSL全体の電子回路構成

AK80-9は単体のモーター（オンボードエンコーダ付き）であるため、周辺電子機器はDephy ActPackではなく直接Raspberry Piに接続する必要がある。制御モードにかかわらず、CAN Busで運用する場合は「Servo Mode」の使用が推奨される。

### 構成図（CANモード、Servo/MITいずれか）

```
[Knee Encoder] [Ankle Encoder]  [6ch Strain Amp] -- Loadcell Wires --> [Load Cell]
        \            |                 |
         \-- I2C Bus -----------------/
                      |
             [Lorde AHRS (IMU)] -- Serial Port
                      |
              [Raspberry Pi 4B] -- SPI --> [SPI to CAN Adapter] -- CAN --+--> [Ankle AK80-9 Motor] -- XT30 --> [Ankle Battery]
                                                                          +--> [Knee AK80-9 Motor]  -- XT30 --> [Knee Battery]
```

### 互換周辺機器（opensourcelegライブラリ対応）

1. AS5048B-TS_EK_AB（14bit磁気エンコーダ）
2. 6ch Strain Amp（Dephy Inc製）
3. Lord Microstrain AHRS（IMU）

### AK80-9モーター1基のセットアップに必要なもの

1. CubeMars AK80-9アクチュエータ ×1
2. Raspberry Pi 4 ×1
3. big PiCAN2 Hat ×1 または CAN Bus plus RS485 Hat ×1（後者は安価・コンパクトだが、性能はやや劣る）
4. モーター設定用のシリアル-USB変換器 ×1
   - CubeMars製「RLink」デバイス（CAN経由でGUIからテスト可能、推奨）
   - 安価なFTDIコンバータ（GUIからのCANテストは不可）
5. 24V〜48Vを供給可能な電源 ×1（バッテリー、デスクトップ電源、AC-DC電源など）
6. 3by1 UARTコネクタ ×1、4by1 CANコネクタ ×1（ドライバチップ用。モーターに付属、自作する場合は専用キットあり）
7. Male XT30 LiPoバッテリー style コネクタ ×1（ドライバチップ用。モーターに付属、自作する場合は専用キットあり）

---

## AK80-9の制御モード

モーターは以下の3モードのいずれかで制御可能。それぞれセットアップ手順・機能・制限が異なる。

- **MIT Mode**: MIT Mini Cheetahコントローラーがベース。CAN Busのみ対応。
- **Servo Mode**: より一般的なモーター制御オプションを公開。CAN Bus・シリアルポート両対応。

CAN通信の方が高速、シリアル通信の方がより多くのフィードバックデータを得られる。MIT Modeは電流制限が低いがオンボードのインピーダンス制御が可能。Servo Modeは電流制限が高いがオンボードインピーダンス制御はない。

**推奨**: 高い最大電流と更新頻度を活かせるため、基本的にはCANポート経由のServo Modeを推奨。より多くのデータが必要ならシリアルポート経由のServo Mode、オンボードインピーダンス制御が必要ならMIT Modeを使用（ただしピークトルクは低下）。

### モード別パラメータ比較表

| 仕様 | MIT Mode (CAN Bus) | Servo Mode (CAN Bus) | Servo Mode (Serial Port) |
|---|---|---|---|
| 最大電流（トルク）指令 | 18 Nm（約16.6A q軸） | 60 A（q軸） | 60 A（q軸） |
| 推奨ピーク電流 | 24 A | 24 A | 24 A |
| 通信速度 | 1 MBPS | 1 MBPS | 962100 Bps |
| 最大更新頻度 | 1000 Hz | 1000 Hz | 50 Hz |
| 必要周辺機器 | CAN to USB変換器 | CAN to USB変換器 | Serial to USB変換器 |
| 電流制御 | 可 | 可 | 可 |
| デューティサイクル（電圧）制御 | 不可 | 可 | 可 |
| オンボードインピーダンス制御 | 可（MIT Mini Cheetahコントローラー） | 不可 | 不可 |
| 台形速度計画によるオンボード位置制御 | 不可 | 可 | 可 |
| 速度制御 | 可（インピーダンス制御経由） | 可（PIDコントローラー経由） | 可（PIDコントローラー経由） |
| 位置・速度・q軸電流・温度・エラーコード報告 | 可 | 可 | 可 |
| q軸電圧、d軸電流・電圧、入力電流・電圧、詳細エラーコード報告 | 不可 | 不可 | 可 |

---

## モーターのキャリブレーションと設定

運用前に、まずシリアルポート経由でセットアップが必要。動画チュートリアルはYoyo LiuのYouTubeチャンネルを参照。

### Servo Mode（CANまたはシリアル）でのキャリブレーション手順

1. Windows PCで、TMotor公式サイトからR-Linkプログラム（「Upper Computer」）をダウンロード。FTDIコネクタでも動作する。
2. モーターの電源をOFFにした状態で、USB-シリアルデバイスをドライバチップのUARTポートとPCに接続。
3. R-Linkプログラムを開く。左下で言語を英語に切替可能。
4. 「refresh」ボタンを押す。左側のメニューにシリアルポート（例: COM6）が表示されることを確認。
5. 「Mode Switch」ボタンを押してモード切替メニューを表示。モーターはデフォルトでMIT Mode設定のため、Servo Modeファームウェアに切り替える。
6. 左サイドバーの「Servo App」ボタンを押す。
7. 数秒待つと切替完了のポップアップが表示される（表示されない場合はCubeMarsにServo Mode対応ファームウェアか確認）。
8. 「Parameter Settings」ボタンを押し、キャリブレーション・パラメータ設定画面を開く。
9. モーターが自由に回転できる状態であることを確認。
10. 「Measure R/L」ボタンを押し、ポップアップでOKを押す。
11. カチカチという音がして完了するとポップアップが表示されるのでOKを押す。
12. 「Measure Lambda」ボタンを押し、ポップアップでOKを押す。
13. モーターが数秒間一定速度で回転し、完了するとポップアップが表示されるのでOKを押す。
14. 右下の「Update」ボタンを押し、測定したパラメータをモーターに書き込む。
15. 「Detect Encoder」欄の「Start」ボタンを押す（画面が小さいとこの欄が正しく表示されないことがあるが、ボタンの位置は同じ）。
16. モーターが回転し、高音のノイズが出るが問題ない。
17. 右上の「Update」ボタンを押す。
18. ゲイン・制限値等のパラメータが希望通りか確認（後からいつでも変更可能）。エンコーダ・電流制御・下部の測定値は自分の環境と異なっていても問題ない。
19. 左サイドバーの「Application Functions」ボタンを押す。
20. コントローラーIDを希望の番号に設定し、Baud rateを「BAUD_RATE_1M」に設定。任意で「Send status over CAN」を有効化してレートを設定し、タイムアウト時間（モーターに使う更新遅延より長めに）を設定。
21. 左サイドバーの「System Settings」ボタンを押す。
22. AK80-9のデフォルト設定と一致しているか確認。
23. 左サイドバーの「Write Parameters」ボタンを押す。成功すると画面右下に緑色で「Parameter write OK」と表示される。
24. 左サイドバーの「Export Settings」ボタンを押し、設定を保存（MCParamsファイルとAppParamsファイルが出力される）。
25. モーターの電源を切るか、GUI右側パネルの「Servo Control」タブでテスト可能。
26. 保存した設定は「Import Settings」から後で読み込み可能。

---

## TMotorCANControlライブラリのインストール

1. [こちら](#)の手順に従ってRaspberry Pi 4をセットアップ（SSH接続がしやすくなる）。
2. Piに対してSSH接続できることを確認。
3. Piに接続後、`pip install TMotorCANControl` を実行してモーター通信用ライブラリをインストール。失敗した場合はGitHubリポジトリからフォークまたは直接コードをダウンロード可能。
4. 上記コマンドで NeuroLocoMiddleware ライブラリ、python-can ライブラリ、pyserial ライブラリも自動インストールされるはず。されない場合は個別に以下を実行:
   ```bash
   pip install NeuroLocoMiddleware
   pip install python-can
   pip install pyserial
   ```
5. チュートリアルで使用するサンプルスクリプトはGitHubリポジトリのdemosフォルダ内にある（TMotorCANControlDemos）。

---

## Raspberry Piの通信設定

### CAN通信（Servo ModeまたはMIT Mode）

1. PiCAN hatには120Ω終端抵抗が搭載されており、「JP3」ラベルのリード線をショートすることで有効化できる。2ピンヘッダをはんだ付けしジャンパを接続する。CAN Bus plus RS485 Hatの場合、この抵抗は既に配線済み。
2. Piの電源をOFFにした状態で、PiCAN HatをPiのGPIOヘッダに接続。
3. PiCAN hatまたはCAN Bus plus RS485 hatのセットアップ手順に従う（車載CANバス向けの記載もあるが、本用途には無関係な部分がある。問題があればトラブルシューティングガイドを確認）。
4. モーター側ドライバボードのCAN High/Low線を、CAN hatの対応するスクリューターミナルに接続。CANポートのピン順序はマニュアル上「Low, High, High, Low」の場合があるが、モーターにより順序が異なることがあり、その際はモーター筐体のラベルに従う。GND端子は、ドライバチップのUARTポートのGNDライン、または電源のGNDに接続し、共通GNDを確保する。
5. CAN接続をテストするには、テストスクリプトを保存したフォルダに移動し、「servo_can」または「mit_can」サブフォルダに入る。
6. `nano check_motor_connection_mit_can.py` または `nano check_motor_connection_servo_can.py` を実行してエディタでスクリプトを開く。
7. 変数「ID」「Type」をモーターのCAN IDおよびモータータイプに合わせて変更する（他の全サンプルスクリプトでも同様。AK80-9でCAN ID=1の場合は変更不要）。
8. nanoを終了するには `Ctrl+X` → `y` → `Enter`。
9. モーターの電源を入れる。
10. `python3 check_motor_connection_mit_can.py` または `python3 check_motor_connection_servo_can.py` を実行。「motor is successfully connected!」と出力されればセットアップ完了。
11. 接続失敗時のチェックリスト:
    1. モーターの電源が入っており、青いLEDが点灯しているか確認。
    2. `check_motor_connection.py` 内のCAN IDとモータータイプ変数が正しいか確認。
    3. 配線を再確認。
    4. PiCAN hatのJP3終端抵抗の接続を確認。
    5. PiCAN hatが「feedback mode」で正しく設定されているか確認。
    6. 必要なPythonモジュールが全てインストールされているか確認。
    7. 解決しない場合はGitHubリポジトリでissueを報告。

---

## TMotorCANControl サンプルスクリプト

サンプルはアクチュエータからのデータ読み取りと、様々な制御則によるアクチュエータ指令を含む。以下の手順で実行する:

1. デモをダウンロードしていない場合はGitHubリポジトリのdemosフォルダから取得（TMotorCANControlDemos）。
2. インストールしたフォルダに移動し、使用する通信・制御モード（MITまたはServo、CANまたはSerial）のサブフォルダに入る。
3. `dir` でスクリプト一覧を確認、または下記リストを参照。
4. 各スクリプトのモーターID・ポート・タイプ等のパラメータを必要に応じて編集（デフォルトIDのAK80-9ならそのままでよい）。編集時は `nano <script>.py` で開き、`Ctrl+X` → `y` → `Enter` で保存終了。
5. モーターの電源を入れる。
6. `python3 <script>.py` でスクリプトを実行。
7. `Ctrl+C` でプログラムを終了。安全にモーターへ電源OFFコマンドが送信される。

### servo_can フォルダ内スクリプト一覧

| スクリプト名 | 内容 |
|---|---|
| `check_motor_connection_servo_can.py` | モーター接続確認用。 |
| `demo_idle_servo_can.py` | 読み取り専用モードでモーター状態をコンソール出力。チップ温度は0の場合あり。 |
| `demo_position_step_servo_can.py` | インピーダンスコントローラーによる一定位置指令。位置・ゲインを変更可能。 |
| `demo_current_step_servo_can.py` | 一定電流指令。値を変えることで加速度を制御できるが、高速回転するため周囲に注意。 |
| `demo_duty_step_servo_can.py` | 一定デューティサイクル指令。値を変えることでモーター速度を変更できる。 |
| `demo_velocity_servo_can.py` | 一定速度指令。義肢制御には有用性が低いが、ドライバチップの機能として利用可能。 |
| `demo_position_tracking_servo_can.py` | サイン波軌道の追従例。目標経路周りで振動する。 |
| `demo_PD_duty_servo_can.py` | デューティサイクルを用いたPD制御による位置制御の例。 |
| `demo_PD_current_servo_can.py` | 電流を用いたPD制御による位置制御の例（電流はトルクに比例するため実質インピーダンス制御）。 |
| `demo_current_chirp_servo_can.py` | 振動トルク指令によるチャープ音再生。 |
| `demo_two_DOF_servo_can.py` | 一方のモーター角度をもう一方に追従させる例。複数モーターの制御ループ設計のモデルとして利用可能。 |

（`mit_can`、`servo_serial` フォルダにも対応するスクリプト群あり）

---

## 実装の詳細

### 全般

- **単位**: TMotorCANControl APIが報告する値はSI単位系。位置は rad、速度は rad/s、加速度は rad/s²、電流は A、トルクは Nm/A。「output（出力）」角度・トルクと「motor-side（モーター側）」角度・トルクの両方に対応する関数が用意されている。output値はギア減速後の値、motor-side値はギア減速前の値。報告される電流はq軸電流の推定値（下記note 4参照）。

### CAN Bus

- **Pythonバージョン**: CANバス管理に使用するpython-canライブラリはPython 3.7以上が必要。TMotorCANControlの利用にはPython 3.7以上へのアップグレードが必須。
- **通信速度**: モーター1基での最速動作確認済み通信速度は2000Hz。ただしこの速度ではプログラムがほぼ常時稼働状態となるため、安定性確保のため1000Hz以下を推奨（特に追加センサー・アクチュエータを制御ループに含む場合）。

### シリアルポート

- **Pythonバージョン**: シリアルバス管理に使用するpyserialライブラリはPython 2・3両対応。
- **OS**: 特定のLinux CANドライバに依存しないため、Windows PCなどpyserial対応の任意のシリアルインターフェースから駆動可能。
- **通信速度**: 最大1000Hzまで問題なく処理可能。ただしこの速度ではTMotor側でパケットがグルーピングされ、実効更新頻度は約100Hzとなる。この問題を避けるため、シリアルモードでは問題解決までは50Hz推奨。

### Servo Mode

- **ピークトルク**: Servo Mode・60A電流制限下での測定最大トルクは34Nm（ただし測定時電流は30Aのみ）。より高い電流を許容すればさらに高くなる可能性がある。ピーク電流が高いほど過熱までの許容時間は短くなるため注意。

### MIT Mode

- **モーター角度のゼロ点設定**: TMotorコントローラーはモーター位置の「ゼロ点」設定機能を持ち、電源が入っている限りゼロ点を保持する。TMotorCANControl APIの `set_zero_position()` 関数で実行可能。実行後、エンコーダ計測のためモーターは約1秒間応答しなくなる。十分な待機なしに新規コマンドを送るとランタイム警告が出る場合がある。
- **ピークトルク**: MIT ModeでのAK80-9の定常状態ピークトルク測定値は14Nm。Dephy ActPackより低いため、より高いトルクが必要な用途にはAK80-64など高トルクTMotorを推奨。
- **q軸電流とトルク定数の表現**: TMotorCANControl APIが報告する電流値はq軸電流。BLDCモーターにおいて同じトルクを生じるDCモーターの電流に相当し、相ごとの値（3つ）ではなく単一の電流値として扱える。Dephy ActPacksもq軸電流を報告しており、高レベル制御器の記述に有用。ただし、AK-series TMotorコントローラーはq軸電流を直接報告せず、代わりにベンチトップテストで測定したトルクより高めの推定トルク値を報告する。AK80-9のq軸トルク定数は0.115Nm/Aと推定され、TMotorCANControl APIの報告電流値はこれに基づき正しいトルクを与えるq軸電流（A）に調整されている。TMotor公式サイトでのAK80-9のトルク定数表記は0.091Nm/A（line-to-line値、q軸トルク定数0.11〜0.12Nm/Aに相当）で、実測範囲とおおむね一致。他のTMotorは未検証だが同様の挙動を想定し、API内に推定トルク定数を設定済み。異なる値を測定した場合は開発元へのフィードバックを推奨。
- **限界付近での動作**: MIT Modeでは、全AK-series TMotorはエンコーダ分解能により±12.5radの位置制限を持つ。さらに各モーターは固有の速度・電流（トルク）制限を持つ。これらの限界に近い動作は非推奨。限界を超えると、TMotorは値を反対側の境界へラップする（例: +12.5radを超えて動くと-12.5radにラップ）。これは紛らわしいため、TMotorCANControl APIはこのラップアラウンド効果を補正し、12.5radや各速度限界を超えた値も測定可能にしている。ただし、駆動側がその値を認識できないため、範囲外の位置・速度を指令することは依然としてできない。限界付近ではAPIがデータを平滑化し、モーター状態が急激に振動して見えないようにする。また、電流も最大/最小値にクランプされる（急変化すると制御不能になり得るため）。上限から下限への瞬時切替は推奨されないが、APIとしては対応可能。
- **トルク変動**: MIT Modeでは、1回転につき1つの磁極角度を通過する際に約20%のトルク低下が2回発生することが観測されている（ローター1回転あたり42回、出力シャフト1回転あたり378回）。テスト機の損傷または校正不良の可能性もある。

---

*本ページは2022年12月16日に最終更新（原文）。日本語訳・Markdown化はChaudeによる作成。*
