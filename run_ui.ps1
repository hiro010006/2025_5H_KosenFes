# ── run_ui.ps1（Qtに触らない確実版）──────────────────
Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# 0) スクリプトの場所に固定（相対パス事故防止）
Set-Location $PSScriptRoot

# 1) venv 作成 & 有効化
if (-not (Test-Path ".\.venv\Scripts\Activate.ps1")) {
  Write-Host "[setup] create venv (.venv)"
  py -3 -m venv .venv
}
. .\.venv\Scripts\Activate.ps1

# 2) 依存チェック
function Install-PythonPackageIfMissing {
  param([Parameter(Mandatory=$true)][string]$Name)
  $info = & pip show $Name 2>$null
  if (-not $info) { Write-Host "[pip] install $Name"; pip install --no-cache-dir $Name }
  else { Write-Host "[pip] ok $Name" }
}
Install-PythonPackageIfMissing numpy
Install-PythonPackageIfMissing PyQt5
Install-PythonPackageIfMissing opencv-python-headless

# 3) Qt プラグインの自動検出（見失った時だけ設定）
# 既存の環境変数は邪魔になるので消す
Remove-Item Env:QT_QPA_PLATFORM_PLUGIN_PATH -ErrorAction SilentlyContinue
Remove-Item Env:QT_PLUGIN_PATH -ErrorAction SilentlyContinue

$site = "$PSScriptRoot\.venv\Lib\site-packages\PyQt5"
$plat = Get-ChildItem "$site\**\platforms\qwindows.dll" -ErrorAction SilentlyContinue | Select-Object -First 1
if ($plat) {
  $env:QT_QPA_PLATFORM_PLUGIN_PATH = $plat.Directory.FullName
  $bin = (Resolve-Path (Join-Path $plat.Directory.Parent.Parent.FullName "bin")).Path
  $env:PATH = "$bin;$env:PATH"
  Write-Host "[qt] plugin=$env:QT_QPA_PLATFORM_PLUGIN_PATH"
}


# 4) 出力バッファ無効化して実行（←手動で動いたのと同じコマンド）
$env:PYTHONUNBUFFERED = "1"
Write-Host "[run] .\.venv\Scripts\python.exe -X faulthandler -u .\UI_proto.py"
.\.venv\Scripts\python.exe -X faulthandler -u .\UI_proto.py
# ─────────────────────────────────────────────────────
