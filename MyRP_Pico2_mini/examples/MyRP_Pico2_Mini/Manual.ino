/* ==========================================================================
   📘  คู่มือคำสั่ง MyRP_Pico2_mini
       (ไฟล์นี้เป็น comment ล้วน ไม่มีผลกับโค้ด)
       กด Ctrl+F แล้วพิมพ์ [1] [2] ... เพื่อกระโดดไปหมวดที่ต้องการ
   --------------------------------------------------------------------------
   [1]  โครงโปรแกรม / ความหมายพารามิเตอร์     [9]  Gyro พื้นฐาน / ล็อกมุม
   [2]  ตั้งค่า (Setting.ino)                    [10] Gyro หมุน / เลี้ยว
   [3]  ตารางคำสั่งทางแยก 'select'              [11] Gyro วิ่งตรง
   [4]  เดินตามเส้น เดินหน้า  ff...              [12] มอเตอร์ / แบตเตอรี่
   [5]  เดินตามเส้น ถอยหลัง  bb...              [13] แขน / เซอร์โว
   [6]  วิ่งโค้ง cl / cr                         [14] เซนเซอร์ / Calibrate
   [7]  เข้ากึ่งกลาง / ออกตัว / เข้าเส้นชัย       [15] Debug (Serial Monitor)
   [8]  หมุน / เลี้ยว ด้วยเซนเซอร์เส้น            [16] เสียง / ไฟ
   ========================================================================== */

/* ==========================================================================
   [1] โครงโปรแกรม / ความหมายพารามิเตอร์
   ========================================================================== */
//
// setup() {  RobotSetup();  Setting();  sw();  SetRobotAngle();  Mission();  }
//
//   speed   ความเร็ว 0-100
//   L, R    ความเร็วล้อซ้าย / ขวา  (-100 ถึง 100, ติดลบ = ถอย)
//   ms      เวลา (มิลลิวินาที)
//   cm      ระยะทาง (เซนติเมตร, ประมาณจากเวลา x ความเร็ว)
//   dist    ค่าเซนเซอร์วัดระยะ analogRead (ยิ่งใกล้ ค่ายิ่งมาก)
//   'p'     ตัวอักษรคำสั่งทางแยก ดูตาราง [3]
//   dir     ทิศสัมบูรณ์ 0 / 90 / 180 / 270 (อ้างอิงตอน resetAngles)
//   มุม     ลบ = ซ้าย, บวก = ขวา

/* ==========================================================================
   [2] ตั้งค่า (Setting.ino)
   ========================================================================== */
// --- มอเตอร์ ---
// set_Freq("Coreless_Motors");             // 20 kHz  (หรือ "coreless")
// set_Freq("DC_Motors");                   // 1 kHz
//
// --- กรองค่า calibrate ---
// clampSensorValueF(min, max);   clampSensorValueB(min, max);   clampSensorValueC(min, max);
//
// --- เซนเซอร์เส้น ---
// RefLineValue(500);                       // threshold เซนเซอร์หน้า-หลัง (0-1000)
// RefCenterLineValue(500);                 // threshold เซนเซอร์คู่กลาง (0-1000)
// TrackLineColor(0);                       // 0 = พื้นขาวเส้นดำ | 1 = พื้นดำเส้นขาว
// Dottedline(0);                           // 0 = ไม่มีเส้นประ | 1 = มีเส้นประ
//
// --- เข้ากึ่งกลาง ---
// set_line_center(1);                      // 0 = วิ่งตรงเข้ากลาง | 1 = เดินตามเส้นเข้ากลาง
// SetToCenterSpeed(20);                    // ความเร็วตอนเข้ากลาง
// set_brake_fc(ff, fc);                    // เวลาเบรก (ms) ตอนเดินหน้า
// set_brake_bc(bf, bc);                    // เวลาเบรก (ms) ตอนถอยหลัง
// set_pid_lf(kp, kd);  set_pid_lb(kp, kd); // PID ของ lf / lb (ค่าเริ่มต้น 0.015, 0.10)
//
// --- ความเร็วเลี้ยว (ใช้กับ select l r q e) ---
// SetTurnSpeed(50);                        // ความเร็วหมุน 'l' 'r'
// TurnSpeedLeft(L, R, ms);                 TurnSpeedRight(L, R, ms);         // 'q' 'e'  TurnLeft / TurnRight
// SetSensorTurnLeftRight(l, r);            // เซนเซอร์หน้าที่ใช้หยุด TurnLeft (F[0..l]) / TurnRight (F[7..r])
// SetSensorTurnLeftRightB(l, r);           // เซนเซอร์หลังที่ใช้หยุด TurnLeftB (B[7..l]) / TurnRightB (B[0..r])
// TurnBackSpeedLeft(L, R, ms);             TurnBackSpeedRight(L, R, ms);     // TurnLeftB / TurnRightB
// TurnSpeedLeftBackF(L, R, ms);            TurnSpeedRightBackF(L, R, ms);    // TurnLeftBackF / TurnRightBackF
// TurnSpeedLeftBackB(L, R, ms);            TurnSpeedRightBackB(L, R, ms);    // TurnLeftBackB / TurnRightBackB
//
// --- โหมดจำกัดความเร็วล้อ ---
// ModeSpdPID(mode, max, min);                                                // PID เส้น
// set_pid_mode2_limit(up, down);           // PID เส้น โหมด 2: -(Speed+down)..(Speed+up) (ค่าเริ่มต้น 20, 20)
// ModeSpdGyro(mode, max, min);             ModeSpdGyro(modeF, modeB, max, min);  // Gyro (แยกหน้า/หลังได้)
//     mode 0 = 0..max   1 = min..max   2 = -Speed..Speed   3 = ..max   4 = 0..Speed (ไม่ถอยล้อ)
// set_gyro_mode2_limit(up, down);          // Gyro โหมด 2: -(Speed+down)..(Speed+up) (ค่าเริ่มต้น 20, 20)
// set_gyro_mode4_limit(up);                // Gyro โหมด 4: 0..(Speed+up) (ค่าเริ่มต้น 20)
//
// --- PID ไจโร ---
// SetGyroTurn(kp, kd, maxSpd, minSpd, smallAngle, stopThr);   // เลี้ยวล้อเดียวเดินหน้า (turndegree)
// SetGyroTurnB(kp, kd, maxSpd, minSpd, smallAngle, stopThr);  // เลี้ยวล้อเดียวถอยหลัง (turndegreeb / turndirectionb)
// SetGyroSpin(kp, kd, maxSpd, minSpd, smallAngle, stopThr);   // หมุนอยู่กับที่ (spindegree)
// SetGyroRun(kp, kd);                      SetGyroRunB(kp, kd);              // วิ่งตรง RunG / RunGB
//
// --- ตำแหน่งเส้น (0-5000, กลาง = 2500) ---
// set_position_line(2500);                 // ตำแหน่งปกติ
// set_position_line_l(500);                set_position_line_r(4500);        // วิ่งโค้ง cl / cr
//
// --- เซนเซอร์วัดระยะ ---
// SetAnalogDistance(28);                   // ขาอนาล็อก A0-A3 (26-29)
//
// --- ตารางตามความเร็ว (ch = SPD_10, SPD_20 ... SPD_100) ---
// setBalanceSpeed(ch, L, R);               // balance เดินหน้า (ล้อไหนแรงกว่า เพิ่มค่าข้างนั้น)
// setBalanceBackSpeed(ch, L, R);           // balance ถอยหลัง
// Set_KP_KD(ch, kp, kd);                   // PID เดินหน้า
// Set_KP_KD_Back(ch, kp, kd);              // PID ถอยหลัง
// SetDelayBreak(ch, f, b);                 // เวลาเบรก (ms) ตอนหยุดที่เส้น (ไถลเลย = เพิ่ม, ถอยเลย = ลด)

/* ==========================================================================
   [3] ตารางคำสั่งทางแยก 'select'  (ตัวสุดท้ายของ ff / bb / *g)
   --------------------------------------------------------------------------
     ตัวอักษร  | ff / bb (เดินตามเส้น)               | *g / *gb (Gyro)
     ---------+-------------------------------------+----------------------------
     's'      | หยุด                                 | หยุด
     'S'      | วิ่งถึงขอบเส้นแล้วหยุด                 | หยุด
     'p' 'P'  | วิ่งผ่าน                               | วิ่งผ่าน
     'c' 'C'  | เข้ากลางแล้วหยุด                       | เข้ากลางแล้วหยุด
     'l' 'L'  | เข้ากลาง + หมุนซ้าย                    | 'l' เข้ากลาง+หมุนซ้าย  'L' หมุนซ้ายทันที
     'r' 'R'  | เข้ากลาง + หมุนขวา                    | 'r' เข้ากลาง+หมุนขวา  'R' หมุนขวาทันที
     'q' 'Q'  | เลี้ยวโค้งซ้าย  (bb = ขวา)             | เลี้ยวโค้งซ้าย  (gb = ขวา)
     'e' 'E'  | เลี้ยวโค้งขวา  (bb = ซ้าย)             | เลี้ยวโค้งขวา  (gb = ซ้าย)
     'a' 'A'  | หมุนซ้ายด้วยเซนเซอร์หลัง               | -
     'd' 'D'  | หมุนขวาด้วยเซนเซอร์หลัง               | -
     'b' 'B'  | เข้ากลางแล้ววิ่งต่อถึงเส้นอีกฝั่ง          | เหมือนกัน
     'g' 'G'  | ปรับตรงด้วย gyro                      | ตัวอื่น ๆ = SetG(100)
   --------------------------------------------------------------------------
     ตัวพิมพ์ใหญ่ (P C Q E G) = เรียก ToFront() / ToBack() จัดตรงเส้นก่อน
   ========================================================================== */

/* ==========================================================================
   [4] เดินตามเส้น เดินหน้า  ff...
   ========================================================================== */
// ff(speed, 'p');                          // วิ่งจนเจอแยกใดก็ได้ (F0 หรือ F7)
// ffc(speed, 'p');      ffc2(speed, 'p');  // เจอแยกตัด (F0 + F7) | ffc2 ยอมรับแยกกว้าง/แคบ
// ffl(speed, 'p');                         // เจอแยกซ้าย (F0)
// ffl2(speed, 'p');                        // เจอแยกซ้ายเต็ม (F0-F4)
// ffr(speed, 'p');                         // เจอแยกขวา (F7)
// ffr2(speed, 'p');                        // เจอแยกขวาเต็ม (F3-F7)
// ffnum(speed, 'p', n); ffn(speed, 'p', n);    // เจอเส้นที่เซนเซอร์ F[n] (0-7)
// ffsum(speed, 'p', sum);                      // ผลรวม F[0]..F[7] >= sum (0-8000)
// ffwhite(speed, 'p');  ffw(speed, 'p');       // วิ่งจนไม่เจอเส้นเลย
// ffblack(speed, 'p');  ffb(speed, 'p');       // วิ่งตรง (ไม่ PID) จนเจอเส้น
// ffblack(L, R, 'p');   ffb(L, R, 'p');
// ff_distance(speed, 'p', dist);   ffd(speed, 'p', dist);     // จนเซนเซอร์วัดระยะถึง dist
// ff_distances(speed, 'p', dist);  ffds(speed, 'p', dist);    // แบบค่อย ๆ ชะลอ
// fftimer(speed, ms);   fftimer(speed, ms, 'p');              // วิ่งตามเวลา
// fft(speed, ms);       fft(speed, ms, 'p');                  // ชื่อย่อของ fftimer
// ffcm(speed, cm);      ffcm(speed, cm, 'p');                 // วิ่งตามระยะทาง
// lf(ms);                                  // PID อยู่กับที่ จัดตัวตรงเส้น
//
// --- กำหนด Kp / Kd เอง (ใส่ต่อจาก speed) ---
// ใช้ได้กับ ff ffc ffc2 ffl ffl2 ffr ffr2 ffnum/ffn ffsum ffwhite/ffw ff_distance/ffd
// และฝั่ง bb... ทุกตัวที่คู่กัน  (ffblack/ffb ไม่ใช้ PID จึงไม่มี)
// ff(speed, kp, 'p');                      // กำหนด Kp อย่างเดียว (Kd ใช้ค่าจากตารางความเร็ว)
// ff(speed, kp, kd, 'p');                  // กำหนดทั้ง Kp และ Kd
// ffn(speed, kp, kd, 'p', n);              // ตัวที่มีพารามิเตอร์ต่อท้าย ใส่ Kp/Kd หลัง speed เหมือนกัน
// ffd(speed, kp, 'p', dist);
// fftimer(speed, kp, ms);       fftimer(speed, kp, kd, ms);       // ตามเวลา (ใส่ 'p' ท้ายสุดได้)
// fftimer(speed, kp, ms, 'p');  fftimer(speed, kp, kd, ms, 'p');
// ffcm(speed, kp, cm);          ffcm(speed, kp, kd, cm);          // ตามระยะทาง (cm เป็นจำนวนเต็ม)
// ffcm(speed, kp, cm, 'p');     ffcm(speed, kp, kd, cm, 'p');
// fft(speed, kp, [kd,] ms [, 'p']);                               // ชื่อย่อ ใช้แบบเดียวกับ fftimer
// * ใส่ Kp เป็นทศนิยม เช่น 0.02  (ใส่ค่าติดลบ = ใช้ค่าจากตาราง)

/* ==========================================================================
   [5] เดินตามเส้น ถอยหลัง  bb...   (ใช้เซนเซอร์หลัง เหมือน ff ทุกอย่าง)
   ========================================================================== */
// bb(speed, 'p');
// bbc(speed, 'p');      bbc2(speed, 'p');
// bbl(speed, 'p');      bbl2(speed, 'p');
// bbr(speed, 'p');      bbr2(speed, 'p');
// bbnum(speed, 'p', n); bbn(speed, 'p', n);
// bbsum(speed, 'p', sum);                      // ผลรวม B[0]..B[7] >= sum (0-8000)
// bbwhite(speed, 'p');  bbw(speed, 'p');
// bbblack(speed, 'p');  bbb(speed, 'p');
// bbblack(L, R, 'p');   bbb(L, R, 'p');
// bb_distance(speed, 'p', dist);   bbd(speed, 'p', dist);
// bbtimer(speed, ms);   bbtimer(speed, ms, 'p');
// bbt(speed, ms);       bbt(speed, ms, 'p');
// bbcm(speed, cm);      bbcm(speed, cm, 'p');
// lb(ms);
// bb(speed, kp, 'p');   bb(speed, kp, kd, 'p');   bbn(speed, kp, kd, 'p', n);   // กำหนด Kp / Kd เอง
// bbtimer(speed, kp, [kd,] ms [, 'p']);    bbcm(speed, kp, [kd,] cm [, 'p']);
// bbt(speed, kp, [kd,] ms [, 'p']);

/* ==========================================================================
   [6] วิ่งโค้ง  cl = เบี่ยงซ้าย, cr = เบี่ยงขวา
       (ตั้ง set_position_line_l / _r ใน Setting ก่อน)
   ========================================================================== */
//   เดินหน้า                         ถอยหลัง
// ffcl(speed, 'p');                 bbcl(speed, 'p');
// ffcr(speed, 'p');                 bbcr(speed, 'p');
// fftimercl(speed, ms [, 'p']);     bbtimercl(speed, ms [, 'p']);
// fftimercr(speed, ms [, 'p']);     bbtimercr(speed, ms [, 'p']);
// ffcmcl(speed, cm [, 'p']);        bbcmcl(speed, cm [, 'p']);
// ffcmcr(speed, cm [, 'p']);        bbcmcr(speed, cm [, 'p']);
// ffdircl(speed, dir [, 'p']);      bbdircl(speed, dir [, 'p']);     // จนหันถึงทิศ dir (0/90/180/270)
// ffdircr(speed, dir [, 'p']);      bbdircr(speed, dir [, 'p']);
//                                   ( [ ] = ใส่หรือไม่ใส่ก็ได้ )
//
// --- กำหนด Kp / Kd เอง (ใส่ต่อจาก speed) ---
// ffcl(speed, kp, [kd,] 'p');                  bbcl / ffcr / bbcr ใช้แบบเดียวกัน
// fftimercl(speed, kp, [kd,] ms [, 'p']);      ...timercr / bbtimercl / bbtimercr แบบเดียวกัน
// ffcmcl(speed, kp, [kd,] cm [, 'p']);         ...cmcr / bbcmcl / bbcmcr แบบเดียวกัน (cm เป็นจำนวนเต็ม)
// ffdircl(speed, kp, [kd,] dir [, 'p']);       ...dircr / bbdircl / bbdircr แบบเดียวกัน (kp ต้องมีจุดทศนิยม เช่น 0.02)

/* ==========================================================================
   [7] เข้ากึ่งกลาง / ออกตัว / เข้าเส้นชัย
   ========================================================================== */
// ToCenter();                                  // เดินหน้าเข้ากลาง (เซนเซอร์คู่กลาง)
// BackCenter();                                  // ถอยหลังเข้ากลาง
// ToFront();    ToBack();                        // จัดตัวตรงเส้น หน้า / หลัง
// gostart(speed);   gostart(L, R);               // ออกจากกล่องเริ่ม
// goend(speed);     goend(L, R);                 // วิ่งเข้าเส้นชัย
// setf(n);   setb(n);   setfc(n);   setbc(n);    // จัดตรงเส้น n รอบ (= balancef / b / fc / bc)
// set_f(n);  set_b(n);  set_fc(n);  set_bc(n);
// TrackSelectF(speed, 'p');   TrackSelectB(speed, 'p');   // สั่ง select ตรง ๆ

/* ==========================================================================
   [8] หมุน / เลี้ยว ด้วยเซนเซอร์เส้น  ([speed] ใส่หรือไม่ใส่ก็ได้)
   ========================================================================== */
// spinl([speed]);      spinr([speed]);       // หมุนอยู่กับที่ไปเส้นถัดไป (เซนเซอร์หน้า)
// spinl2([speed]);     spinr2([speed]);
// spinlb([speed]);     spinrb([speed]);      // ใช้เซนเซอร์หลัง
// spinl2b([speed]);    spinr2b([speed]);
// TurnLeft();          TurnRight();          // เลี้ยวโค้ง เซนเซอร์หน้า
// TurnLeftB();        TurnRightB();        // เลี้ยวโค้ง เซนเซอร์หลัง
// TurnLeftBackF();     TurnRightBackF();     // เลี้ยวล้อเดียวถอยหลัง เช็คเซนเซอร์หน้า
// TurnLeftBackB();     TurnRightBackB();     // เลี้ยวล้อเดียวถอยหลัง เช็คเซนเซอร์หลัง

/* ==========================================================================
   [9] Gyro พื้นฐาน / ล็อกมุม
   ========================================================================== */
// resetAngles();       // ตั้งมุมปัจจุบัน = 0 (เรียกตอนหุ่นหยุดนิ่ง)
// SetRobotAngle();     // จำมุมปัจจุบันเป็นมุมเป้าหมายวิ่งตรง
// gyroZ();             // อ่านมุม -180..180 (ซ้าย = ลบ, ขวา = บวก)
// SetDirectionG(dir);  relativeToDirection(dir);    // ตั้งทิศเป้าหมายแบบสัมบูรณ์
//
// SetG(ms);    setg(ms);       // ล็อกมุม (หมุนอยู่กับที่) ตามเวลา
// SetGB(ms);   setgb(ms);      // ล็อกมุม แบบถอยหลัง
// SetFG(ms);   setfg(ms);      // ล็อกมุม แบบเดินหน้า
// SetBG(ms);   setbg(ms);      // ล็อกมุม แบบถอยหลัง (จำมุม ณ ตอนเรียก เหมือน SetFG)
// SetHoldAngle();              // จำมุมปัจจุบันให้ HoldAngle...
// HoldAngle();   HoldAngleB();   HoldAngleF();   HoldAngleBF();    // ใช้ใน loop เพื่อค้างมุม

/* ==========================================================================
   [10] Gyro หมุน / เลี้ยว  (มุม: ลบ = ซ้าย, บวก = ขวา | [speed] ใส่หรือไม่ใส่ก็ได้)
   ========================================================================== */
// --- หมุน/เลี้ยวตามมุม ---
// spindegree([speed,] -90);          // หมุนอยู่กับที่
// turndegree([speed,] 90);           // เลี้ยวล้อเดียว เดินหน้า
// turndegreeb([speed,] 90);          // เลี้ยวล้อเดียว ถอยหลัง
// turndegree_none([speed,] 90);      // เหมือน turndegree แต่ไม่เบรก (ต่อคำสั่งถัดไปลื่น)
// turndegreeb_none([speed,] 90);
// rotatedegree(L, R, 90);            rotatedegree(L, R, 90, kp, kd);   // โค้งกว้าง กำหนดล้อเอง
//
// --- หันไปทิศสัมบูรณ์ (0 / 90 / 180 / 270) ---
// spindirection([speed,] dir);
// turndirection([speed,] dir);
// turndirectionb([speed,] dir);
// turndirection_none([speed,] dir);
// turndirectionb_none([speed,] dir);
//
// --- วิ่งตามเส้นจนหันถึงทิศ (ใช้กับเส้นโค้ง) ---
// ffdir(speed, dir [, 'p']);             bbdir(speed, dir [, 'p']);             // เช่น ffdir(50, 270);
// ffdir(speed, kp, [kd,] dir [, 'p']);   bbdir(speed, kp, [kd,] dir [, 'p']);   // กำหนด Kp / Kd เอง (kp ต้องมีจุดทศนิยม)
// set_line_dir_threshold(5.0);           // ระยะยอมรับ (องศา) ค่าเริ่มต้น 5
//
// --- ชื่อย่อ (ใส่มุมเป็นบวกเสมอ) ---
//   ซ้าย                ขวา                 ย่อ
// spinlg([speed,] 90);  spinrg([speed,] 90);  slg / srg      // หมุนอยู่กับที่
// turnlg([speed,] 90);  turnrg([speed,] 90);  tlg / trg      // เลี้ยวเดินหน้า
// turnlbg([speed,] 90); turnrbg([speed,] 90); tlbg / trbg    // เลี้ยวถอยหลัง
//
// --- เลี้ยว 2 จังหวะต่อเนื่อง (ไม่หยุดกลางทาง) ---
// tlrg([speed,] 90 [, 45]);    trlg([speed,] 90 [, 45]);     // ซ้ายแล้วขวา / ขวาแล้วซ้าย
// tlrbg([speed,] 90 [, 45]);   trlbg([speed,] 90 [, 45]);    // แบบถอยหลัง

/* ==========================================================================
   [11] Gyro วิ่งตรง  ( [ ] = ใส่หรือไม่ใส่ก็ได้ | ใส่ dir ท้าย = ล็อกทิศสัมบูรณ์ )
   ========================================================================== */
//   เดินหน้า                              ถอยหลัง
// fftimerg(speed, ms [, 'p'] [, dir]);   bbtimerg(speed, ms [, 'p'] [, dir]);   // ตามเวลา
// fftg(speed, ms, 'p' [, dir]);          bbtg(speed, ms, 'p' [, dir]);          // ชื่อย่อ
// ffcmg(speed, cm [, 'p'] [, dir]);      bbcmg(speed, cm [, 'p'] [, dir]);      // ตามระยะ (เร่ง/ผ่อน)
// ffcmgs(speed, cm [, 'p'] [, dir]);     bbcmgs(speed, cm [, 'p'] [, dir]);     // ตามระยะ (ความเร็วคงที่)
// ffbg(speed, 'p' [, dir]);              bbbg(speed, 'p' [, dir]);              // วิ่งจนเจอเส้น
// ffdg(speed, 'p', dist);                bbdg(speed, 'p', dist);                // จนเซนเซอร์วัดระยะถึง dist
// ffdgs(speed, 'p', dist);               bbdgs(speed, 'p', dist);               // แบบค่อย ๆ ชะลอ
// ToCenterG();                           BackCenterG();                         // เข้ากลางด้วย gyro
// ToFrontG();                            ToBackG();                             // วิ่งจนเจอเส้น หน้า / หลัง
// TrackSelectG(speed, 'p');              TrackSelectGB(speed, 'p');             // สั่ง select ตรง ๆ
// RunG(L, R);                            RunGB(L, R);                           // 1 รอบ PID ใช้ใน loop เอง

/* ==========================================================================
   [12] มอเตอร์ / แบตเตอรี่
   ========================================================================== */
// Motor(L, R);                 // สั่งล้อซ้าย/ขวาค้างไว้
// Move(L, R, ms);              // สั่งล้อตามเวลา
// MotorStop();   MotorStop(ms);
// MotorShot();   MotorShot(ms, power);       // เบรกช็อต (power 0-100)
// fd(speed, ms);   bk(speed, ms);            // เดินหน้า / ถอยหลัง
// sl(speed, ms);   sr(speed, ms);            // หมุนซ้าย / ขวา
// tl(speed, ms);   tr(speed, ms);            // เลี้ยวซ้าย / ขวา (ล้อเดียว)
// getBatteryVoltage();                       // อ่านแรงดันแบต (V)

/* ==========================================================================
   [13] แขน / เซอร์โว
   ========================================================================== */
// Servo(pin, angle);                         // pin: 0, 1, 10, 28
// Servo(a, b, c);              Servo(a, b, c, speed);
// armupdown(angle [, speed]);                // แขนขึ้น-ลง
// arm_left_right(l, r [, speed]);            // มือซ้าย-ขวา
// S0_trim(x);   S1_trim(x);   S10_trim(x);   S28_trim(x);     // ชดเชยองศา
// SerialServoControl();                      // ปรับผ่าน Serial พิมพ์ "10 90"
//
// --- ท่าสำเร็จรูปใน Servo.ino (ใส่ speed ได้ เช่น arm_close(2);) ---
// arm_ready();       arm_up();          arm_up45();         arm_down();
// arm_open();        arm_close();       arm_open_l();       arm_open_r();
// arm_open_down();   arm_down_open();   arm_open_up();      arm_up_open();
// arm_down_close();  arm_close_down();  arm_up_close();     arm_close_up();
// arm_big();         arm_big_box();     arm_big_box_up();   arm_behihd();

/* ==========================================================================
   [14] เซนเซอร์ / Calibrate
   ========================================================================== */
// --- ตัวแปร ---
// F[0-7]  B[0-7]  C[0-1]           // ค่าเซนเซอร์ หน้า / หลัง / กลาง (หลัง calibrate 0-1000)
// Ref  RefC                        // threshold หน้า-หลัง / กลาง
//
// --- อ่านค่า ---
// ReadCalibrateF();   ReadCalibrateB();   ReadCalibrateC();     // อ่านแบบ calibrate แล้ว
// ReadF();            ReadB();            ReadC();              // อ่านค่าดิบ
// ReadSensor();       ReadSensorRaw();
//
// --- Calibrate ---
// calibrateA();        calibrateB();        calibrateC();
// saveCalibA_LOCAL();  saveCalibB_LOCAL();  saveCalibC_LOCAL();
// loadCalibration();   loadCalibration_LOCAL();   printCalibration_LOCAL();

/* ==========================================================================
   [15] Debug (Serial Monitor) — ค้าง while(1) ใช้ใน Setting() เท่านั้น
   ========================================================================== */
// Serial_FrontSensor();            Serial_BackSensor();             // ค่าดิบ
// Serial_CenterSensor();           Serial_AllSensor();
// SerialCalibrate_FrontSensor();   SerialCalibrate_BackSensor();    // ค่า calibrate
// SerialCalibrate_CenterSensor();  SerialCalibrate_AllSensor();
// SerialPositionF();   SerialPositionB();   SerialPositionFB();     // ตำแหน่งเส้น
// SerialDistance();                                                 // เซนเซอร์วัดระยะ

/* ==========================================================================
   [16] เสียง / ไฟ
   ========================================================================== */
// Beep(ms);          Beep2(freq, ms);     beep(freq, ms);
// BeepScanner();                          // ปี๊บสั้นแบบเครื่องสแกนบาร์โค้ด
// BZon();            BZoff();             // เปิด / ปิด buzzer ค้าง
// blink(n);                               // กระพริบไฟ n ครั้ง
