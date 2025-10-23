# ── run_ui.ps1 ─────────────────────────────────────────────
# 1) venv 作成 & 有効化
if (-not (Test-Path ".\.venv\Scripts\Activate.ps1")) {
  Write-Host "[setup] create venv (.venv)"
  py -3 -m venv .venv
}
. .\.venv\Scripts\Activate.ps1

# 2) 依存チェック（pip show で判定）
function Install-PythonPackageIfMissing {
  [CmdletBinding()]
  param(
    [Parameter(Mandatory=$true)][string]$Name
  )
  $info = & pip show $Name 2>$null
  if (-not $info) {
    Write-Host "[pip] install $Name"
    pip install --no-cache-dir $Name
  } else {
    Write-Host "[pip] ok $Name"
  }
}

Install-PythonPackageIfMissing -Name numpy
Install-PythonPackageIfMissing -Name PyQt5
Install-PythonPackageIfMissing -Name opencv-python-headless

# Qt plugin パス保険（Qt5優先→Qt）
$base = Join-Path $PSScriptRoot ".venv\Lib\site-packages\PyQt5\Qt5"
if (-not (Test-Path $base)) { $base = Join-Path $PSScriptRoot ".venv\Lib\site-packages\PyQt5\Qt" }
$env:QT_QPA_PLATFORM_PLUGIN_PATH = (Join-Path $base "plugins\platforms")
$env:PATH = (Join-Path $base "bin") + ";" + $env:PATH
Remove-Item Env:QT_PLUGIN_PATH -ErrorAction SilentlyContinue


# 3) 起動（QtパスはUI_proto.py側で自動解決する構成）
python .\UI_proto.py
# ───────────────────────────────────────────────────────────
