# 作業ログ：Raspberry Pi 5 + Waveshare 2-CH CAN HAT 環境構築記録

### 参考文献

https://www.switch-science.com/products/9777?srsltid=AfmBOoq4zuWBeCkPBUubRyZQAw3v9eg9XZJJMMs7mv_DEayi9cm-iLCo

https://www.waveshare.com/wiki/2-CH_CAN_HAT+

**作成日:** 2026/02/13

**目的:** TMotor (AK80-9) 制御のためのCAN通信環境の構築

**ハードウェア:** Raspberry Pi 5 (8GB)

**OS:** Raspberry Pi OS (Bookworm) 64-bit

**デバイス:** Waveshare 2-CH CAN HAT

## 1. 概要と注意点

Raspberry Pi 5 および OS (Bookworm) 環境では、Waveshare公式Wikiの古い手順（Pi 4以前向け）がそのままでは通用しない箇所が多数ある。

本ログは、Pi 5向けに手順を修正・最適化したものである。

**主な変更点:**

1. **WiringPi:** 公式手順のバージョン(2.70)は動作しないため、GitHubの有志版(3.x)を使用。
2. **Python環境:** システムへの `pip install` は禁止(PEP 668)されているため、`uv` による仮想環境を使用。
3. **設定ファイル:** `config.txt` の場所が `/boot/firmware/` に変更されている。

## 2. 下位レイヤー（C言語ライブラリ）の導入

### 2-1. bcm2835 ライブラリ

公式Wikiの手順通り、ソースコードからビルドする。

- コンパイル時にポインタキャストの警告が出るが、動作に影響はないため無視してよい。
- `make check` でテストがPASSすることを確認済み。

### 2-2. WiringPi (※重要：Pi 5対応版への置換)

Waveshare公式リンクのバージョンは古く、Pi 5のCPU情報を取得できずエラー(`Oops: Unable to determine board revision`)となる。

必ず以下のGitHub版を使用すること。

```
# 古いフォルダがある場合は削除
cd ~
rm -rf WiringPi-master

# Pi 5対応版をクローンしてビルド
git clone [https://github.com/WiringPi/WiringPi.git](https://github.com/WiringPi/WiringPi.git)
cd WiringPi
./build
```

**確認:** `gpio -v` を実行し、バージョンが **3.x系** であることを確認する。

## 3. Python仮想環境の構築 (uv + TMotorCANControl)

### 3-1. 環境構築

`uv` を使用してプロジェクトごとの仮想環境を作成する。

- **プロジェクトパス:** `~/Research/y25m060_20260212/TMotorCANControl`

```
cd ~/Research/y25m060_20260212/TMotorCANControl
uv sync
source .venv/bin/activate
```

### 3-2. 依存ライブラリのインストール (Pi 5互換対応)

`RPi.GPIO` はPi 5のハードウェア構造に対応していないため、互換ライブラリ **`rpi-lgpio`** を使用する。

```
# 仮想環境内 (.venv) で実行
uv pip install rpi-lgpio  # RPi.GPIOの代替
uv pip install spidev pillow python-can numpy
```

### 3-3. 開発ツールの導入 (AIコーディング対応)

コード品質維持のため、Linter/Formatterとして `ruff` を導入。

VS Codeの「保存時自動整形」を有効化済み。

```
uv add --dev ruff
```

## 4. ハードウェア設定 (config.txt)

### 4-1. オーバーレイの設定

Pi 5では設定ファイルの場所が変更されている点に注意。

また、本HATは標準のSPI0ではなく、**SPI1** を使用する特殊な構成である。

**編集コマンド:** `sudo nano /boot/firmware/config.txt`

**追記内容:**

```
# Waveshare 2-CH CAN HAT Configuration
dtparam=spi=on
dtoverlay=i2c0
dtoverlay=spi1-3cs
dtoverlay=mcp2515,spi1-1,oscillator=16000000,interrupt=22
dtoverlay=mcp2515,spi1-2,oscillator=16000000,interrupt=13
```

※ 追記後は `sudo reboot` が必須。

### 4-2. 自動起動設定 (rc.local)

再起動するとCANインターフェースがDOWN状態に戻るため、起動時に自動でUPするように設定する。

**編集コマンド:** `sudo nano /etc/rc.local`

**追記場所:** `exit 0` の行より前

**追記内容:**

```
# CAN通信の有効化 (Bitrate: 1Mbps for AK80-9)
ip link set can0 up type can bitrate 1000000
ip link set can1 up type can bitrate 1000000
ifconfig can0 txqueuelen 65536
ifconfig can1 txqueuelen 65536
```

**（補足）**

- 現在は `rc.local` を使用しているが、将来的に不安定な場合は `systemd` ユニット化を推奨。
- `txqueuelen` は 65536 としているが、モータ制御のリアルタイム性を優先する場合は `1000` 程度まで下げて遅延を抑制する。

## 5. 動作検証 (ループバックテスト)

### 5-1. 物理配線 (テスト時のみ)

1台のHATで送受信テストを行う場合、以下のピンを直結する。

- `CAN0 H` ⇔ `CAN1 H`
- `CAN0 L` ⇔ `CAN1 L`
- **注意:** 基板上の120Ω抵抗スイッチを **ON** にすること。

### 5-2. テストコマンド

`can-utils` を使用して疎通確認を行う。

- **受信側 (Terminal 1):**
    
    ```
    candump can0
    ```
    
- **送信側 (Terminal 2):**
    
    ```
    cansend can1 000#11.22.33.44
    ```
    

**結果:** 受信側に `11 22 33 44` が表示されれば正常。

(ハードウェア、ドライバ、配線すべてOK)

### **5-3. トラブルシューティング**

- 通信が途絶した場合は `ip -d -s link show can0` を実行。`ERROR-SETTING` や `bus-off` が出ていないか確認。
- `uv` 仮想環境で `sudo` が必要なスクリプトを実行する場合は、`.venv/bin/python` を直接指定すること。

## 6. 安全運用ルール

モータ制御時の事故防止のため、以下の手順を厳守すること。

1. **電源投入順序:**
    - Raspberry Pi 起動 (CANインターフェースUP確認) → モータ電源ON
2. **電源遮断順序:**
    - モータ電源OFF → Raspberry Pi シャットダウン
3. **緊急停止 (E-Stop):**
    - モータ用電源ライン（24V）に物理的な緊急停止スイッチを必ず配置すること。
    - プログラム暴走時はPC操作ではなく、物理スイッチで電源を遮断する。
4. **配線:**
    - ループバックテスト用のジャンパー線は、モータ接続前に必ず取り外すこと。