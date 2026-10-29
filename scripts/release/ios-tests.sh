#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
mkdir -p build/ios-tests
# Keep the installed runtime; downloading one here would hide missing runner setup.
runtimes=$(xcrun simctl list runtimes -j)
runtime=$(RUNTIMES="$runtimes" python3 -c 'import json,os; r=[x for x in json.loads(os.environ["RUNTIMES"])["runtimes"] if x.get("isAvailable") and x["identifier"].startswith("com.apple.CoreSimulator.SimRuntime.iOS-") and tuple(map(int,x["version"].split("."))) >= (18,5)]; assert r,"An iOS 18.5+ simulator runtime is required"; print(max(r,key=lambda x:tuple(map(int,x["version"].split("."))))["identifier"])')
core_tested=false
cleanup() {
  local result=$?
  if [[ "$result" != 0 && "${CI:-}" == true ]]; then
    mkdir -p build/ios-tests/diagnostics
    xcrun simctl spawn "$udid" log show --last 5m --style compact \
      --predicate 'process == "Hanppie" OR process == "SimMetalHost"' \
      > "build/ios-tests/diagnostics/$slug-runtime.log" 2>&1 || true
    for directory in "$HOME/Library/Logs/DiagnosticReports" \
      "$HOME/Library/Developer/CoreSimulator/Devices/$udid/data/Library/Logs/DiagnosticReports"; do
      [[ -d "$directory" ]] || continue
      for report in "$directory"/Hanppie*.ips "$directory"/SimMetalHost*.ips; do
        [[ -f "$report" ]] || continue
        cp "$report" build/ios-tests/diagnostics/ || true
      done
    done
  fi
  xcrun simctl shutdown "$udid" >/dev/null 2>&1 || true
  xcrun simctl delete "$udid" >/dev/null 2>&1 || true
  exit "$result"
}
# Newer runtimes drop older device types; fail with the supported names instead of simctl's bare error.
for device in 'iPhone 17' 'iPad Pro 11-inch (M5)'; do
  type=$(RUNTIMES="$runtimes" RUNTIME="$runtime" DEVICE="$device" python3 -c 'import json,os,sys; r=next(x for x in json.loads(os.environ["RUNTIMES"])["runtimes"] if x["identifier"]==os.environ["RUNTIME"]); t=[x for x in r["supportedDeviceTypes"] if x["name"]==os.environ["DEVICE"]]; t or sys.exit("%s is not supported by %s; supported: %s" % (os.environ["DEVICE"], r["name"], ", ".join(x["name"] for x in r["supportedDeviceTypes"]))); print(t[0]["identifier"])')
  udid=$(xcrun simctl create "Hanppie $device" "$type" "$runtime")
  slug=$(echo "$device" | tr ' ()' '---')
  trap cleanup EXIT
  xcrun simctl boot "$udid"
  xcrun simctl bootstatus "$udid" -b
  if [[ "$core_tested" == false ]]; then
    ./gradlew :packages:robot-core:iosSimulatorArm64Test --device "$udid"
    core_tested=true
  fi
  # xcresult retains logs and screenshots; skip device-wide sysdiagnoses.
  xcodebuild -project iosApp/Hanppie.xcodeproj -scheme Hanppie -configuration Debug \
    -destination "platform=iOS Simulator,id=$udid" -derivedDataPath build/ios-derived \
    -resultBundlePath "build/ios-tests/$slug.xcresult" \
    -parallel-testing-enabled NO -collect-test-diagnostics never CODE_SIGNING_ALLOWED=NO test
  xcrun simctl shutdown "$udid"
  xcrun simctl delete "$udid"
  trap - EXIT
done
