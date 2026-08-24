[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

$currentSkillRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$currentSkillsDir = Split-Path $currentSkillRoot -Parent
$currentAgentsDir = Split-Path $currentSkillsDir -Parent
$scopeRoot = Split-Path $currentAgentsDir -Parent
$allowedRepos = @('flight_control', 'car_control', 'camera_helper')
$isInstalledLayout = ((Split-Path $currentSkillsDir -Leaf) -eq 'skills') -and
    ((Split-Path $currentAgentsDir -Leaf) -eq '.agents')

if($isInstalledLayout -and ((Split-Path $scopeRoot -Leaf) -in $allowedRepos))
{
    $suiteRoot = [System.IO.Path]::GetFullPath((Split-Path $scopeRoot -Parent))
}
elseif($isInstalledLayout)
{
    $suiteRoot = [System.IO.Path]::GetFullPath($scopeRoot)
}
else
{
    Write-Host 'Standalone skill source: no Motherboard_Demo mirror layout detected.'
    return
}

$canonicalRoot = [System.IO.Path]::GetFullPath(
    (Join-Path $suiteRoot '.agents\skills\feiyue-leiqu'))

if(-not (Test-Path -LiteralPath (Join-Path $canonicalRoot 'SKILL.md')))
{
    $legacyRoot = [System.IO.Path]::GetFullPath(
        (Join-Path $suiteRoot 'flight_control\.agents\skills\feiyue-leiqu'))
    if(Test-Path -LiteralPath (Join-Path $legacyRoot 'SKILL.md'))
    {
        $canonicalRoot = $legacyRoot
    }
    else
    {
        throw "Canonical skill not found: $canonicalRoot"
    }
}

$runtimeFiles = @(
    'SKILL.md',
    'VERSION',
    'agents\openai.yaml',
    'references\onboarding.md',
    'references\system-contracts.md',
    'references\flight-architecture.md',
    'references\mission-development.md',
    'references\flight-control.md',
    'references\car-control.md',
    'references\vision-and-helper.md',
    'references\diagnosis-playbook.md',
    'references\validation-matrix.md',
    'references\evolution-protocol.md',
    'scripts\inspect-suite.ps1',
    'scripts\audit-skill.ps1',
    'scripts\sync-mirrors.ps1'
)

foreach($relativePath in $runtimeFiles)
{
    if(-not (Test-Path -LiteralPath (Join-Path $canonicalRoot $relativePath)))
    {
        throw "Canonical runtime file missing: $relativePath"
    }
}

foreach($repoName in $allowedRepos)
{
    $repoRoot = [System.IO.Path]::GetFullPath((Join-Path $suiteRoot $repoName))
    $suitePrefix = $suiteRoot.TrimEnd('\') + '\'

    if(-not $repoRoot.StartsWith($suitePrefix, [System.StringComparison]::OrdinalIgnoreCase))
    {
        throw "Resolved repository escaped suite root: $repoRoot"
    }
    if(-not (Test-Path -LiteralPath (Join-Path $repoRoot '.git')))
    {
        Write-Host "SKIP $repoName (repository not present)"
        continue
    }

    $targetRoot = Join-Path $repoRoot '.agents\skills\feiyue-leiqu'
    foreach($relativePath in $runtimeFiles)
    {
        $sourceFile = Join-Path $canonicalRoot $relativePath
        $targetFile = Join-Path $targetRoot $relativePath
        $targetDirectory = Split-Path $targetFile -Parent

        if(-not (Test-Path -LiteralPath $targetDirectory))
        {
            New-Item -ItemType Directory -Path $targetDirectory -Force | Out-Null
        }
        Copy-Item -LiteralPath $sourceFile -Destination $targetFile -Force
    }

    Write-Host "SYNCED $repoName"
}
