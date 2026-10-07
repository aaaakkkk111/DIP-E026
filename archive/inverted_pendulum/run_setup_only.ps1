$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot

$venvPython = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $venvPython)) {
    $pythonCommand = $null
    $pythonArguments = @()
    if (Get-Command py -ErrorAction SilentlyContinue) {
        $pythonCommand = (Get-Command py).Source
        $pythonArguments = @("-3.12")
    } elseif (Get-Command python -ErrorAction SilentlyContinue) {
        $pythonCommand = (Get-Command python).Source
    } else {
        throw "Python was not found. Install 64-bit Python 3.12 and enable Add python.exe to PATH."
    }
    & $pythonCommand @pythonArguments -m venv .venv
}
& $venvPython -m pip install --disable-pip-version-check -r requirements-lock.txt
& $venvPython check_env.py
