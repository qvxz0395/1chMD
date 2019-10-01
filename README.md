# MD
## 要求仕様
  - 1ch
  - RZ735VA500W駆動可能
  - 出来ればPWM周波数50kHz(maxon motor controllerと同じ値)
  - 正転逆転
  - 電流制御
  - 速度制御
  - AMT102対応
  - CAN通信
  - 超小型,ターミナル方式

## 部品
  - メインマイコン
    - [STM32F042K6T6](http://akizukidenshi.com/catalog/g/gI-10789/)
    - [データシート](https://www.st.com/resource/en/datasheet/stm32f042c4.pdf) 
    - [リファレンスマニュアル](https://www.aps-web.jp/magazine/st/data/STM32F0x1_2_8RM_RM0091_JPN_rev1.pdf)

      - 250YEN
      - CAN 
      - 電流検出ADC12Bit
      - 書き込みデバッグ JTAG
      - ロータリーエンコーダ入力
      - とりあえず生やしとくピン
        - 4PDIPスイッチ
        - SPI1ch
        - USART1ch

  - CANトランシーバ
    - [ATA6561-GAQW-N](https://www.digikey.jp/product-detail/ja/microchip-technology/ATA6561-GAQW-N/ATA6561-GAQW-NCT-ND/9453180)
      - 63YEN
      - SOIC08
      - 電源 5V
      - 

  - 電流センサ
    - [ACS781KLRTR-150U-T](https://www.digikey.jp/product-detail/ja/allegro-microsystems-llc/ACS781KLRTR-150U-T/620-1851-1-ND/6677748)
      - 605YEN
      - ホール効果での電流検出
      - 検出可能電流 0~100[A] 過渡的に150[A], 17.6mV/A
      - 絶対最大定格 285[A],25℃
      - 

  - FET
    - [FDBL86561-F085](https://www.digikey.jp/product-detail/ja/on-semiconductor/FDBL86561-F085/FDBL86561-F085CT-ND/5209201)
      - 450 YEN @1
      - ドレインソース間電圧$V_{dss}$= 60[V]
      - ドレインソース間連続電流( $V_{gs}$ = 10[V], $T_c$ = 25[℃]) $I_D$= 300[A]
      - $I_d,V_{gs}$印加時の$R_{ds} On$= 1.1[mΩ] (max) @ 80[A] , 10[V]
      - $V_{gs}$印加時のゲート電荷 $Q_g$= 220[nC] (max) @10[V]
      - パッケージ　8-PowerSFN
    - 似た特性,同じパッケージのFET
      - [IPT007N06NATMA1](https://www.digikey.jp/product-detail/ja/infineon-technologies/IPT007N06NATMA1/IPT007N06NATMA1CT-ND/4571878)
      - [FDBL0110N60](https://www.digikey.jp/product-detail/ja/on-semiconductor/FDBL0110N60/FDBL0110N60CT-ND/5216101)

  - ゲートドライバ
    - [A3921KLPTR-T](http://akizukidenshi.com/catalog/g/gI-12293/)
      - 350YEN
      - フルブリッジMOSFETドライバ


  - 3.3Vレギュレータ
    - [NJM2845DL1-33](http://akizukidenshi.com/catalog/g/gI-11299/)
      - 50YEN
      - 最大入力電圧 14[V]
      - 出力電流　0.8[A]

  - 5Vレギュレータ
    - [NJM7805SDL1](http://akizukidenshi.com/catalog/g/gI-11237/)
      - 30YEN
      - 最大入力電圧 35[V]
      - 出力電流 1.5[A]

  - デジタルアイソレータ
    - [ADuM7440](https://www.digikey.jp/product-detail/ja/analog-devices-inc/ADUM7440ARQZ-RL7/ADUM7440ARQZ-RL7CT-ND/3897187)
      - 605YEN
      - 電源電圧 3-5.5[V]
      - 4Ch
      - データレート 1[Mbps]

  - モード変更用DIPSW
    - [EHS104LD](http://akizukidenshi.com/catalog/g/gP-05064/)

  - ショットキーバリアダイオード
    - 
## 計算(パワー回路設計)
  - [TeXコマンドをmarkdownでつかう方法](https://qiita.com/kgoto/items/2452b6dbaed5b3f7df78)

### ゲート抵抗
  - [参考](https://detail-infomation.com/gate-driver-gate-resistance-design/)

  FDBL86561-F085 のデータシートの表 Electrical Characteristics より
  
  Total Gate Charge $Q_g$ = 170[nC] (typ.) @ 10V 

  これより，

  ゲート駆動電圧 = 10[V]

  A3921KLPTR-T のデータシート表 ELECTRICAL CHARACTERISTICS Pulldown On Resistance 内の $I_{GHx} , I_{GLx}$より
  
  ゲート電流$I_G$= 150[mA]
  
  ゲート駆動回路の出力電圧$V_{OUT}= V_{REG} = 12.8[V]$より，(A3921の電気特性表より)

  ゲート抵抗$R_G$は

  $$ R_G = \dfrac{V_{out}}{I_G} = \dfrac{12.8[V]}{0.15[A]} = 85[Ω] $$



  ところで，
  電流iを時間tで積分したものが電荷$Q$なので，

  $$Q = \int idt$$

  ゲート電流$i_G$が時間$t_{ON}$流れることで，ゲートに蓄えられた電荷量が$Q_g$になる．

  $$Q_g = \int_0^{t_{ON}} I_G dt = I_G [t]_0^{t_{ON}} = I_G t_{ON}$$

  上記の関係より，

  $$ t_{ON} = \dfrac{Q_g}{t_{ON}} = \dfrac{170[nC]}{0.15[A]} = 1134[ns] $$

### デッドタイム

### チャージポンプコンデンサ

### ブートストラップコンデンサ

### 正逆転確認LEDの電流制限抵抗

### FFの電流制限抵抗

### 5V電源の電流制限抵抗

### 周波数

### 発熱

FETの発熱を考える．([参考](https://lab.fujiele.co.jp/articles/3129/))
他で発熱を考える必要があるのはゲートドライバICだけど，今回は無視．

損失は主に2種類ある．「オン抵抗と電流」によるものと「スイッチング損失」によるもの．

#### オン抵抗と電流

  $P = I^2R$

#### スイッチング損失

  FETはスイッチング時に損失が発生する．損失の式は．

  $ああああああ$



## マイコン関連



### ロータリーエンコーダ
  - [参考(STM32＋HALでエンコーダ―モード)](https://garberas.com/archives/244)

  - [参考(3.3V-5Vレベル変換)](http://kosakai.world.coocan.jp/change_3_3V_5V.html)

#### 回路
  よく使うインクリメンタルエンコーダ[AMT102](https://www.cui.com/product/resource/amt10.pdf)は電源$VDD=5$[V]駆動で,
  パルス信号出力も5[V]である．
  
  しかし，マイコンは3.3V駆動である．

  そのため，マイコンのピンの耐圧が5V以下だった場合，出力信号を[分圧](http://www.kairo-nyumon.com/resistor_divider.html)して電圧を落とさないといけない．

  マイコンの耐圧を確認する．今回の設計ではPA6とPB5,PB6が，エンコーダに接続されている．  
  データシートのP33からあるTable.13 pin definitionによると，それぞれのピンのI/O structureは順にTTa,FT,FTfである．  
  Table.12によると，TTaの耐圧が3.3V, FT,FTfの耐圧が5Vとなっている．また，Table18の絶対最大定格にはFT,FTfも内部プルアップ，プルダウンをすると**耐圧が下がる**と注3に書いてあるので，気を付ける．
  
  PA6だけは分圧必須である．（もう発注しちゃった，，，2019/08/21回路図及び配線図を修正）

#### プログラム

リファレンスマニュアルp341～にエンコーダインターフェースモードについての説明がある．  
