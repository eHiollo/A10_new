# The kaanh controller's daily password

How the "Manufacturer" password on a kaanhbot-based controller (the A10 arm) is computed,
how it was established, and what in this repository implements it.

Scope: this is about reaching the *controller's own* privileged commands on hardware the
project owns. The password is a credential: keep it out of tracked files, logs and chat -
the A10 repository already has two of them in its git history.

## What the controller does

`kaanhbot::user::UserManagement` (in `kaanhbot-2.5.7/lib/release/libkaanhbot_lib.so`) has six
roles and, for two of them, a *dynamic* password instead of a stored one:

| Role | id | password |
|---|---|---|
| `None` / `Operator` / `Engineer` / `Administrator` | 0..3 | stored (`resetPwd`), and the dynamic path returns the constant `md5("0")` |
| **Manufacturer** | 4 | `md5(str(year + month + day))` |
| **Developer** | 5 | `md5(str(year + month + day + hour))` |

All in **local time** - the implementation calls `localtime_r` on the controller's own clock.

## The function

`kaanhbot::user::UserManagement::Imp::getDynamicPwd(User)`, exported as
`_ZN8kaanhbot4user14UserManagement3Imp13getDynamicPwdB5cxx11ENS0_4UserE`, file offset
`0x4aeab0`. Read out of the disassembly:

```asm
; prologue: the returned std::string is in rdi (sret), the role in esi
mov    %rdi,%r12
call   std::chrono::system_clock::now()
call   localtime_r                      ; struct tm at rsp+0x20
cmp    $0x4,%ebx ; je  Manufacturer     ; role 4
cmp    $0x5,%ebx ; je  Developer       ; role 5
...
Manufacturer:
mov    0x34(%rsp),%edx                  ; tm_year (years since 1900)
mov    0x30(%rsp),%eax                  ; tm_mon  (0-based)
lea    0x76d(%rdx,%rax,1),%r14d         ; 1901 + tm_year + tm_mon
add    0x2c(%rsp),%r14d                 ; + tm_mday
...
Developer: (+ tm_hour at 0x28(%rsp))
...
call   kaanh::general::md5(std::string const&)
```

`1901 + (year - 1900) + (month - 1) + day` is exactly **`year + month + day`**, and the
integer is converted to decimal (`std::to_string`) before hashing. The other roles reach a
stub that hashes the single character `"0"`.

Two consequences worth knowing:

* the password changes at local midnight, and a controller whose clock or timezone is wrong
  computes a different one;
* because the input is a *sum*, different dates collide (`2026-07-24` and `2026-09-22` both
  give `2057`), which is why a password from one day can still be valid on another.

## How it was established

1. **Disassembly** of the shipped SDK (above), which named the roles and the arithmetic.
2. **The vendor's own function as ground truth** - `tools/check_passport_sdk.py` loads
   `libkaanhbot_lib.so` and calls `getDynamicPwd` for every role, then compares it with this
   repository's derivation:

   ```
   OK   Manufacturer   sdk=2eace51d8f796d04991c831a07059758 ours=2eace51d8f796d04991c831a07059758
   OK   Developer      sdk=54ff9e9e3a2ec0300d4ce11261f5169f ours=54ff9e9e3a2ec0300d4ce11261f5169f
   OK   Administrator  sdk=cfcd208495d565ef66e7dff9f98764da ours=cfcd208495d565ef66e7dff9f98764da
   ...
   agreement
   ```

   (The SDK ships `libethercat.so.1.2.0` without the `libethercat.so.1` symlink the linker
   looks for; the tool preloads the dependency by path instead of editing the SDK.)
3. **The passwords recorded in the A10 repository itself**, which the scheme reproduces:
   `a_passport.txt` was created on **2026-05-26** carrying
   `df4fe8a8bcd5c95cdb640aa9793bb32b` = `md5("2057")` = `md5(str(2026+5+26))`, and the value
   written a few days before the 2026-06-10 commit, `a48564053b3c7b54800246348c7fa4a0` =
   `md5("2039")`. Both are pinned in `tests/test_a10_login.py`.

## What this repository implements

| File | What |
|---|---|
| `rl/kaanh_passport.py` | the derivation (`daily_password`, `daily_sum`, `login_command`) plus a CLI |
| `rl/aris_cmd.py` | the controller's command channel: `login`, `policy`, anything else |
| `rl/a10_env.py` | `login()` / `start_policy()` / `send_command()` on the A10 transport |
| `tools/check_passport_sdk.py` | cross-checks the derivation against the vendor library |
| `tests/test_a10_login.py` | pins the derivation to the historical values and the SDK |

```bash
python3 rl/kaanh_passport.py                      # today's Manufacturer password
python3 rl/kaanh_passport.py --command            # login --user=Manufacturer --pwd=...
python3 rl/kaanh_passport.py --user Developer     # includes the hour
python3 rl/kaanh_passport.py --date 2026-06-07    # what it was on that day
python3 tools/check_passport_sdk.py --allow-skip  # the vendor's own answer, if the SDK is here
```

## Using it

The login line goes to the controller's command interface, not to the action port (see
`docs/A10.md` for that channel). With the A10 transport:

```bash
python3 rl/run_policy.py --transport a10 --a10-host <robot> --a10-login --a10-start-policy \
    --a10-arm-vel 1.5 --a10-limits -3:3,-3:3,-3:3,-3:3,-3:3,-3:3,0:100 --chunk 16 --steps 200
```

`--a10-login` derives today's password locally; nothing is sent anywhere except the login
command itself. Add `--a10-cmd-port` if the controller's command interface is not on 5866
(it is configured in `kaanh.xml` - `WebInterface`/`HttpInterface`, and the client tries raw
TCP first, then a websocket).

## Caveats

* **It is a credential.** The Manufacturer role reaches every privileged command (`m_init`,
  `ptr`, calibration, program start). Do not write the output into a repository, a log, or a
  ticket; `a_passport.txt` in the A10 repo is exactly the anti-pattern (and its two values
  are still in that repository's history).
* **Obfuscation, not authentication.** Anyone who knows this scheme can log in on any day,
  and the value collides across dates. Treat the controller's own authentication as a
  convenience lock and keep the network perimeter (see the unaudited `:8080` note in
  `docs/A10.md`) as the real boundary.
* **Clock and timezone matter.** The check runs on the controller's `localtime`; a drift of a
  day or a wrong `TZ` makes the password wrong.
* If the machine was delivered by the vendor, replacing or bypassing their controller may
  touch warranty or support terms. That is a commercial decision, not a technical one.
