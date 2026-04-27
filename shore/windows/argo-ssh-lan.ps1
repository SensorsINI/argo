# Resolve Argo on LAN from neighbor table; see ..\README.md for the fixed WiFi MAC.
#Requires -Version 5.1
param(
    [string] $ArgoMac = "C8-26-E2-6C-58-BA",
    [string] $SshUser = "orangepi",
    # Strongly recommended: restrict neighbor lookup to the active LAN/Wi-Fi interface.
    # Example: -InterfaceAlias "Wi-Fi"
    [string] $InterfaceAlias = "",
    # Optional: if set, only accept neighbor IPs that share the same first N octets
    # as the local interface IPv4 (typical: 2 for 10.1.*.*, 3 for 192.168.1.*)
    [int] $MatchPrefixOctets = 0
)

$mac = $ArgoMac -replace "-", ""

function Get-ActiveInterface {
    param([string] $Alias)
    if (-not [string]::IsNullOrWhiteSpace($Alias)) {
        $cfg = Get-NetIPConfiguration -InterfaceAlias $Alias -ErrorAction Stop
        $ip = ($cfg.IPv4Address | Select-Object -First 1).IPv4Address
        if ([string]::IsNullOrWhiteSpace($ip)) { throw "Interface '$Alias' has no IPv4 address." }
        return [pscustomobject]@{ Alias = $Alias; Index = $cfg.InterfaceIndex; IPv4 = $ip }
    }

    # Prefer the interface that owns the IPv4 default route (0.0.0.0/0)
    $defaultIfIndex = $null
    try {
        $defaultIfIndex = (Get-NetRoute -DestinationPrefix "0.0.0.0/0" -ErrorAction Stop |
            Sort-Object -Property RouteMetric, InterfaceMetric |
            Select-Object -First 1 -ExpandProperty InterfaceIndex)
    } catch { $defaultIfIndex = $null }

    $cfg = $null
    if ($defaultIfIndex) {
        $cfg = Get-NetIPConfiguration -InterfaceIndex $defaultIfIndex -ErrorAction SilentlyContinue
    }

    if (-not $cfg) {
        $cfg = Get-NetIPConfiguration | Where-Object {
            $_.IPv4DefaultGateway -and $_.IPv4Address -and $_.InterfaceOperationalStatus -eq "Up"
        } | Select-Object -First 1
    }

    # Last-resort fallback: try Wi-Fi if it has an IPv4 address (some networks don't report a default gateway cleanly)
    if (-not $cfg) {
        $cfg = Get-NetIPConfiguration -InterfaceAlias "Wi-Fi" -ErrorAction SilentlyContinue
        if (-not ($cfg -and $cfg.IPv4Address)) { $cfg = $null }
    }

    if (-not $cfg) { throw "Could not find an active IPv4 interface with a default gateway." }
    $alias = $cfg.InterfaceAlias
    $ip = ($cfg.IPv4Address | Select-Object -First 1).IPv4Address
    return [pscustomobject]@{ Alias = $alias; Index = $cfg.InterfaceIndex; IPv4 = $ip }
}

$iface = $null
try { $iface = Get-ActiveInterface -Alias $InterfaceAlias }
catch {
    [Console]::Error.WriteLine("Interface selection failed: $($_.Exception.Message)")
    [Console]::Error.WriteLine("Tip: run: Get-NetIPConfiguration | Select InterfaceAlias,IPv4Address,IPv4DefaultGateway")
    exit 1
}

$prefix = $null
if ($MatchPrefixOctets -gt 0) {
    $p = ($iface.IPv4 -split "\.") | Select-Object -First $MatchPrefixOctets
    $prefix = ($p -join ".") + "."
}

$neighbors = Get-NetNeighbor -AddressFamily IPv4 -InterfaceIndex $iface.Index
$rows = $neighbors | Where-Object {
    if (-not $_.LinkLayerAddress) { return $false }
    if (($_.LinkLayerAddress -replace "-", "") -ine $mac) { return $false }
    if ($_.IPAddress -notmatch "^\d{1,3}(\.\d{1,3}){3}$") { return $false }
    if ($prefix -and -not ($_.IPAddress.StartsWith($prefix))) { return $false }
    $first = [int]($_.IPAddress -split "\.")[0]
    if ($first -lt 1 -or $first -gt 223) { return $false }
    $true
}
if (-not $rows) {
    [Console]::Error.WriteLine(
        "Argo not found on interface '$($iface.Alias)' (IPv4 $($iface.IPv4)): no neighbor with MAC $ArgoMac. " +
        "Is Argo connected to the same Wi-Fi/LAN and is client-to-client traffic allowed? " +
        "Check with: Get-NetNeighbor -AddressFamily IPv4 -InterfaceAlias ""$($iface.Alias)"" | Select IPAddress,LinkLayerAddress,State"
    )
    exit 1
}
$order = "Reachable", "Stale", "Permanent", "Probe", "Delay", "Unreachable"
$ip = $rows | Sort-Object {
    $t = $_.State.ToString()
    $i = [array]::IndexOf($order, $t)
    if ($i -lt 0) { 8 } else { $i }
} | Select-Object -First 1 -ExpandProperty IPAddress
if ([string]::IsNullOrWhiteSpace($ip)) {
    [Console]::Error.WriteLine("Could not pick an IP for that MAC.")
    exit 1
}
[Console]::WriteLine("Argo at $ip - starting SSH as $SshUser ...")
$target = "$SshUser@$ip"
$sshExit = 0
& ssh -o "ConnectTimeout=15" $target
if ($LASTEXITCODE -ne $null) { $sshExit = $LASTEXITCODE }
elseif ($? -eq $false) { $sshExit = 1 }

if ($sshExit -ne 0) {
    [Console]::Error.WriteLine("")
    [Console]::Error.WriteLine("SSH failed (exit $sshExit). Checking whether Argo is pingable...")

    $pingOk = $false
    try {
        $pingOk = Test-Connection -ComputerName $ip -Count 1 -Quiet -ErrorAction Stop
    } catch {
        $pingOk = $false
    }

    if ($pingOk) {
        [Console]::Error.WriteLine("Ping OK: $ip responds. SSH is likely the issue.")
        [Console]::Error.WriteLine("Try:")
        [Console]::Error.WriteLine("  - Verify Argo sshd is running:  sudo systemctl status ssh")
        [Console]::Error.WriteLine("  - Check port 22 reachable:      ssh -vvv $target")
        [Console]::Error.WriteLine("  - First-time host key prompt:   run from an interactive terminal (not backgrounded)")
        [Console]::Error.WriteLine("  - Firewall/port change:         confirm ssh listens on 22 (ss -lntp | grep ':22')")
    } else {
        [Console]::Error.WriteLine("Ping FAILED: $ip does not respond.")
        [Console]::Error.WriteLine("Try:")
        [Console]::Error.WriteLine("  - Ensure you're on the same WiFi LAN as Argo")
        [Console]::Error.WriteLine("  - Power-cycle Argo or wait for WiFi reconnect")
        [Console]::Error.WriteLine("  - Refresh neighbor table: ping the subnet gateway, then re-run this script")
        [Console]::Error.WriteLine("  - Re-check mapping: Get-NetNeighbor -AddressFamily IPv4 | Select IPAddress, LinkLayerAddress, State")
    }

    exit $sshExit
}

exit 0
