# 同志社大学 関西春ロボ2025の中央制御プログラム
機体の制御、CAN通信による各モジュールとの通信、Bluetoothによるモバイル機器との通信を行います。

## Enviroments
- PlatformIO
    - Core 6.1.17
    - Home 3.3.4
    - Board manager: Espressif ESP32 Dev Module
    - Framework: Arduino
- ESP32-WROOM-32E

## プログラムについて
### はじめに
このプログラムは、ArduinoIDEではなく、PlatformIOとC++20で書かれています。PlatformIOの利点はたくさんありますが、最も大きな恩恵の一つは**機能別のファイルの分割**ができるようになることです。  
機能別にファイルやディレクトリを分けることで、プログラムが整理され、見たいプログラムもすぐに見つけることができます。

また、このプログラムではただ機能ごとにソースファイルを分けるのではなく、プログラムをローカルライブラリとして分割しています。そのためsrcフォルダ内にはエントリポイントとなる`main.cpp`しかほとんど存在しません。  
このようにすることで、各機能のプログラムは他のプログラムへの依存が最小限・明確になり、機能ごとのテストが容易になります。

プログラム内のほとんどのクラスや関数には**Docコメント**("///"から始まるコメント)がつけられています。Docコメントが付けられたクラスや関数は、エディタ上でマウスホバーすることで、そこに記述された概要や使い方を見ることができます。  
(ソースコードを読んでいてコメントがない場合は、定義部ではなく宣言部(.hppファイルなど)に書かれている可能性があります。マウスホバーしてみてコメントが表示されるならどこかに書いてあるはずです。)

### プロジェクト構成
プログラムの構成は以下のようになっています。

```
.
├── lib/
│   ├── # ローカルライブラリ(各機能のプログラム)
│   ├── bt_communication/
│   │   ├── include/
│   │   │   ├── # 外部からインクルードするヘッダファイル
│   │   │   └── ...
│   │   └── src/
│   │       ├── # ソースファイル(プログラムの中身)
│   │       └── ...
│   ├── can/
│   │   └── ...
│   ├── m3508_control/
│   │   └── ...
│   └── vec2/
│       └── ...
├── src/
│   └── main.cpp  # エントリポイント
├── test/
│   ├── # テスト用プログラム
│   └── ...
└── # 各種設定ファイルなど…
```

`lib`ディレクトリには各機能のプログラムが入っています。`src`ディレクトリにはmain.cpp(エントリポイント)などのプロシージャルなコードが入っています。

### プログラムを読むコツ
このプログラムはC++のオブジェクト指向の機能をふんだんに使って書かれており、全てを理解するのは大変です。ここで大切なのは、オブジェクト指向とは、プログラムを抽象化することで、**呼び出し側(利用者側)のコード**を分かりやすく、簡単な記述で済ませる手法であるということです。

オススメはエントリポイントの`main.cpp`から読み始めることです。コードは抽象化されているので、例えば`cat.meow()`や`world.execute()`のように、コードの中身は分からなくてもどんな動作をするのかは想像がつく記述が多いです。  
まずはプログラムの全体の処理の流れを確認して、それから必要に応じて具体的な処理の実装を確認するのが良いと思います。

C++の文法は基本的にArduinoと同じですが、見たことのない記述や文法が出てきた場合は、Googleで検索してみましょう。もしも外部ライブラリを使っている場所が分からない場合は、インターネット上の記事も参考にしつつ、**公式ドキュメント**や**そのライブラリのソースコード**も確認してみましょう。

Qiitaなどのインターネット記事は、アウトラインの理解には大いに助けになりますが、細かい仕様に関する記述などは公式ドキュメントにしか書かれていないことがあります。また、最近は公式ドキュメントに**丁寧なチュートリアル**が記載されていることも多いです。  
公式ドキュメントは英語で書かれていることが多いですが、Google翻訳やDeeplを使えば読むことができます。  

ライブラリのソースコードはGithubで公開されている場合が多いほか、エディタ上からも[右クリック → 定義へ移動]などで確認できます。ソースコード上に書いてあるコメントが参考になることも多いので、積極的に活用していきましょう。

### 注意
- コメントやドキュメントは最新のコードの状況を反映していないことがあります。
- コードはC++の経験の浅い人間が書いたもので、非効率的な書き方やバッドプラクティスが存在する場合があります。