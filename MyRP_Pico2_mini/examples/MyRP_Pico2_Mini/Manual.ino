/* =====================================================
   📘 คู่มือคำสั่งไลบรารี MyRP_Pico2_mini
   ไฟล์นี้เป็น comment ล้วน ไม่มีผลต่อการคอมไพล์

   เริ่มต้นใน setup():
     RobotSetup();   // เริ่มระบบ (Serial, I2C, ADC, โหลดค่า calibrate, IMU)
     Setting();      // ตั้งค่าพารามิเตอร์ของหุ่น
     sw();           // เมนูคาลิเบรตเซนเซอร์ผ่านปุ่มกด
     Mission();      // โค้ดภารกิจ
   ===================================================== */


/* =====================================================
   🛠️ SETTING.INO — คู่มือฟังก์ชันตั้งค่าทั้งหมด
   (เรียงตามลำดับที่ปรากฏใน Setting.ino)
   ===================================================== */

// ── มอเตอร์ ──────────────────────────────────────────
// set_Freq("Coreless_Motors" หรือ "coreless")   // มอเตอร์คอร์เลส -> PWM 20kHz (เงียบ)
// set_Freq("DC_Motors" หรือค่าอื่น ๆ)             // มอเตอร์ DC ทั่วไป -> PWM 1kHz (ค่าเริ่มต้น)

// ── คาลิเบรตเซนเซอร์ (ช่วงค่าหลังคาลิเบรต 0-1000) ──────
// clampSensorValueF(min, max);   // ค่านอกช่วงนี้ (เซนเซอร์หน้า) จะถูกปัดเป็น 0 หรือ 1000
// clampSensorValueB(min, max);   // เหมือนกันแต่เป็นเซนเซอร์หลัง
// clampSensorValueC(min, max);   // เหมือนกันแต่เป็นเซนเซอร์คู่กลาง
// ตัวอย่าง: clampSensorValueF(100, 800);  // ต่ำกว่า 100 -> 0, สูงกว่า 800 -> 1000

// ── ตั้งค่าเซนเซอร์เส้น ────────────────────────────────
// RefLineValue(x);         // threshold ตัดสินว่า "เจอเส้น" ของเซนเซอร์แถวหน้า-หลัง (0-1000)
// RefCenterLineValue(x);   // threshold ตัดสินว่า "เจอเส้น" ของเซนเซอร์คู่กลาง (0-1000)
// TrackLineColor(0);       // 0 = พื้นขาวเส้นดำ | 1 = พื้นดำเส้นขาว
// Dottedline(0);           // 0 = เส้นทึบปกติ | 1 = สนามมีเส้นประ (กันหลุดเส้นตอนเจอช่องว่าง)

// ── โหมดเข้ากึ่งกลางทางแยก ─────────────────────────────
// set_line_center(0);        // เดินธรรมดาแล้วเข้ากึ่งกลางหุ่น (ไม่สนเส้น)
// set_line_center(1);        // เดินตามเส้น PID แล้วค่อยเข้ากึ่งกลางหุ่น
// SetToCenterSpeed(speed);   // ความเร็วที่ใช้ตอนเข้ากึ่งกลาง (ToCenter/ToCenterL/ToCenterR ฯลฯ)

// ── ความเร็วเลี้ยว/หมุน ────────────────────────────────
// SetTurnSpeed(speed);              // ความเร็ว default ของ spinl()/spinr() (เมื่อไม่ระบุ speed เอง)
// TurnSpeedLeft(l, r, delay);       // ความเร็วล้อซ้าย/ขวา + เวลาหน่วง ตอน TurnLeft() (เลี้ยว q/Q)
// TurnSpeedRight(l, r, delay);      // เหมือนกันแต่ TurnRight() (เลี้ยว e/E)
// TurnBackSpeedLeft(l, r, delay);   // เหมือน TurnSpeedLeft แต่ใช้ตอนถอยหลัง (TurnLeft_B)
// TurnBackSpeedRight(l, r, delay);  // เหมือน TurnSpeedRight แต่ใช้ตอนถอยหลัง (TurnRight_B)
// TurnSpeedLeftBackF(l, r, delay);  // ความเร็วล้อเดียวถอยหลัง เช็คเส้นด้วยเซนเซอร์หน้า F[] ตอน TurnLeftBackF()
// TurnSpeedRightBackF(l, r, delay); // เหมือนกันแต่ TurnRightBackF()
// TurnSpeedLeftBackB(l, r, delay);  // ความเร็วล้อเดียวถอยหลัง เช็คเส้นด้วยเซนเซอร์หลัง B[] ตอน TurnLeftBackB()
// TurnSpeedRightBackB(l, r, delay); // เหมือนกันแต่ TurnRightBackB()

// ── โหมดควบคุมความเร็ว PID ──────────────────────────────
// ModeSpdPID(mode, max, min);   // mode: โหมดคุมความเร็ว | max/min: ขอบเขตความเร็วที่ PID ปรับได้
// SetPIDDeadBand(db);           // error ต่ำกว่านี้ (หน่วยเดียวกับ set_position_line) มองเป็น 0 กันสั่น (default 20)

// ── ตำแหน่งเส้นเป้าหมาย (0-7000, กลางแท่นเซนเซอร์ ≈ 3500) ──
// set_position_line(pos);     // ตำแหน่งเดินตรงกลาง | ต่ำ=เข้าซ้าย สูง=เข้าขวา
// set_position_line_l(pos);   // ตำแหน่งใช้ตอนวิ่งโค้งซ้าย (ffcl/bbcl ฯลฯ)
// set_position_line_r(pos);   // ตำแหน่งใช้ตอนวิ่งโค้งขวา (ffcr/bbcr ฯลฯ)

// ── เซนเซอร์วัดระยะ ────────────────────────────────────
// SetAnalogDistance(pin);   // กำหนดขา analog (A0-A3) ที่ต่อเซนเซอร์วัดระยะ

// ── DEBUG / SERIAL MONITOR (เรียกใน Setting() ได้ทีละตัว, เป็น while(1) ห้ามใช้ใน Mission()) ──
// SerialDistance();                  // อ่านค่าเซนเซอร์วัดระยะทาง Serial
// Serial_FrontSensor();              // ค่าดิบเซนเซอร์หน้า
// Serial_BackSensor();               // ค่าดิบเซนเซอร์หลัง
// Serial_CenterSensor();             // ค่าดิบเซนเซอร์กลาง
// Serial_AllSensor();                // ค่าดิบเซนเซอร์รวมทุกตัว
// SerialCalibrate_FrontSensor();     // ค่าหลังคาลิเบรต (0-1000) เซนเซอร์หน้า
// SerialCalibrate_BackSensor();      // ค่าหลังคาลิเบรต (0-1000) เซนเซอร์หลัง
// SerialCalibrate_CenterSensor();    // ค่าหลังคาลิเบรต (0-1000) เซนเซอร์กลาง
// SerialCalibrate_AllSensor();       // ค่าหลังคาลิเบรตรวมทุกตัว
// SerialPositionF();                 // ตำแหน่งเส้นที่คำนวณได้ (หน้า) เทียบกับ set_position_line
// SerialPositionB();                 // ตำแหน่งเส้นที่คำนวณได้ (หลัง)
// SerialPositionFB();                // ตำแหน่งเส้นที่คำนวณได้ (หน้า-หลัง พร้อมกัน)

// ── RobotSetupSpeed() : ตารางความเร็วต่อช่วง (เรียกจาก Setting()) ──
// ดัชนีช่วงความเร็ว: SPD_10, SPD_20, SPD_30, SPD_40, SPD_50, SPD_60, SPD_70, SPD_80, SPD_90, SPD_100
//
// setBalanceSpeed(ch, l, r);        // ชดเชยความแรงมอเตอร์ซ้าย/ขวาตอนเดินหน้า ที่ช่วงความเร็ว ch (ข้างไหนแรงกว่า เพิ่มค่าข้างนั้น)
// setBalanceBackSpeed(ch, l, r);    // เหมือนกันแต่ตอนถอยหลัง
// Set_KP_KD(ch, kp, kd);            // ค่า PID (Kp, Kd) เดินตามเส้น ที่ช่วงความเร็ว ch
// Set_KP_KD_Back(ch, kp, kd);       // ค่า PID (Kp, Kd) เดินตามเส้นตอนถอยหลัง ที่ช่วงความเร็ว ch
// ตัวอย่าง: Set_KP_KD(SPD_60, 0.018, 0.20);  // ตั้ง Kp=0.018, Kd=0.20 ให้ความเร็วช่วง 60


/* =====================================================
   7️⃣ ตารางคำสั่งเมื่อเจอแยก (select Command Table) — ฉบับละเอียด
   ใช้กับพารามิเตอร์ตัวสุดท้ายของฟังก์ชันตระกูล ff, bb และ *g (เวอร์ชัน gyro) ทั้งหมด
   อ้างอิงพฤติกรรมจริงจาก TrackSelectF/TrackSelectB (MyRP_Pico2_mini_PID.h)
   และ TrackSelectG/TrackSelectGB (MyRP_Pico2_mini_Gyro.h)
   =====================================================
 *
 *   ---------- (A) ตระกูลปกติ (เส้น PID): ff* → TrackSelectF | bb* → TrackSelectB ----------
 *
 *   's' : เบรกหยุดทันที ไม่รอเช็คเซนเซอร์ (ถือว่าอยู่ตรงจุดหยุดพอดีแล้ว)
 *          ff: ถอยสั้น ๆ (Move -15→-10→-1) แล้ว MotorStop()
 *          bb: เดินหน้าสั้น ๆ (Move 15→10→1) แล้ว MotorShot() (เบรกกระชากด้วย back-EMF แรงกว่า)
 *   'S' : วิ่ง PID ตามเส้นต่อไปก่อนจนกว่าเซนเซอร์หน้า/หลังจะเจอขอบเส้น (F[0]/F[7] หรือ B[0]/B[7]) แล้วค่อยเบรกแบบเดียวกับ 's'
 *   'p' || 'P' : วิ่งทะลุผ่านทางแยก/เส้นขวางด้วยความเร็ว spd ตรง ๆ (ไม่ใช้ PID) จนสองเซนเซอร์ริมพ้นเส้น (เช็คซ้ำ 2 รอบกันสัญญาณรบกวน) มีเสียง buzzer ระหว่างวิ่งผ่าน
 *                'P' ต่างจาก 'p' แค่เรียก ToFront()/ToBack() วิ่งเข้าหาเส้นด้วย PID ก่อนเริ่มนับเงื่อนไข
 *   'c' || 'C' : เข้ากึ่งกลางทางแยกแล้วหยุด (ToCenter()/BackCenter()) | 'C' เรียก ToFront()/ToBack() ก่อน
 *   'l' || 'L' : เข้ากึ่งกลางทางแยกแล้วหมุนซ้าย 90° (spinl()) — พิมพ์เล็ก/ใหญ่ทำงาน "เหมือนกันทุกประการ" ในตระกูลนี้ (case ถูก fall-through รวมกัน)
 *   'r' || 'R' : เข้ากึ่งกลางทางแยกแล้วหมุนขวา 90° (spinr()) — พิมพ์เล็ก/ใหญ่เหมือนกัน
 *   'q' || 'Q' : ff = เลี้ยวโค้งซ้าย (แตะขอบซ้าย F[0] ด้วยความเร็วครึ่งหนึ่งของ tctL/tctR ก่อน แล้ว TurnLeft())
 *                bb = เลี้ยวโค้ง "ขวา" (แตะขอบขวา B[7] แล้ว TurnRight_B())  ⚠️ ทิศสลับกับ ff!
 *   'e' || 'E' : ff = เลี้ยวโค้งขวา (แตะขอบขวา F[7] แล้ว TurnRight())
 *                bb = เลี้ยวโค้ง "ซ้าย" (แตะขอบซ้าย B[0] แล้ว TurnLeft_B())  ⚠️ ทิศสลับกับ ff!
 *                ('Q'/'E' ต่างจากตัวพิมพ์เล็กแค่เรียก ToFront()/ToBack() ก่อนเข้าเงื่อนไขแตะขอบ)
 *   'a' || 'A' : เข้ากึ่งกลางแล้วหมุนซ้ายด้วยเซนเซอร์หลัง (spinl_B()) — ทำงานเหมือนกันทั้ง ff/bb (ไม่สลับทิศแบบ q/e)
 *   'd' || 'D' : เข้ากึ่งกลางแล้วหมุนขวาด้วยเซนเซอร์หลัง (spinr_B()) — ทำงานเหมือนกันทั้ง ff/bb
 *   'b' || 'B' : เข้ากึ่งกลาง (ModeToCenter/ModeToCenterBack) แล้ววิ่งต่อจนเซนเซอร์ "ฝั่งตรงข้ามทิศทางวิ่ง" เจอเส้น
 *                ff เช็ค B[] (หลัง) | bb เช็ค F[] (หน้า) แล้วสะบัดเบรกสั้น ๆ ไปทิศตรงข้ามการวิ่ง
 *   'g' || 'G' : เรียก SetFG(100) ปรับตรงมุมด้วย gyro ค้างไว้ 100ms | 'G' เรียก ToFront()/ToBack() ก่อน
 *   อื่น ๆ (default) : MotorStop(20) หยุดพร้อม beep 20ms
 *
 *   ---------- (B) ตระกูล gyro (RunG/RunGB): *g → TrackSelectG | *g...B → TrackSelectGB ----------
 *
 *   'L' : หมุนทันที 90° ซ้ายด้วย gyro (spindegree(-90)) โดย "ไม่" เข้ากึ่งกลางก่อน — G และ GB เรียกเหมือนกันทุกตัวอักษร ไม่กลับทิศ
 *   'l' : เข้ากึ่งกลางทางแยกด้วย gyro ก่อน (ToCenterLG()/BackCenterG()) แล้วค่อยหมุนซ้าย 90°
 *   'R' : หมุนทันที 90° ขวาด้วย gyro (spindegree(90)) ไม่เข้ากึ่งกลางก่อน
 *   'r' : เข้ากึ่งกลางด้วย gyro ก่อนแล้วค่อยหมุนขวา 90°
 *         ⚠️ ต่างจากตระกูล (A): ที่นี่ตัวพิมพ์เล็ก 'l'/'r' (เข้ากึ่งกลางก่อนค่อยหมุน) กับตัวพิมพ์ใหญ่ 'L'/'R' (หมุนทันที ไม่เข้ากึ่งกลาง)
 *         มีความหมาย "ต่างกันจริง" ไม่เหมือนตระกูล (A) ที่พิมพ์เล็ก/ใหญ่เท่ากัน — ระวังเผลอใช้ตัวพิมพ์ผิด
 *   'q' || 'Q' : G  = เลี้ยวโค้งซ้าย (turndegree(-90))
 *                GB = เลี้ยวโค้ง "ขวา" (turndegreeb(90))  ⚠️ ทิศสลับกับ G เหมือน pattern ของตระกูล (A)
 *   'e' || 'E' : G  = เลี้ยวโค้งขวา (turndegree(90))
 *                GB = เลี้ยวโค้ง "ซ้าย" (turndegreeb(-90))  ⚠️ ทิศสลับกับ G
 *   'p' || 'P' : วิ่งทะลุผ่านทางแยกด้วย gyro (RunG/RunGB) จนพ้นเส้น 2 รอบ แทรก fftimerg/bbtimerg สั้น ๆ (5ms) กลางทาง มีเสียง buzzer
 *                พิมพ์เล็ก/ใหญ่เหมือนกัน — ไม่มี ToFront()/ToBack() นำหน้าแบบตระกูล (A)
 *   'c' || 'C' : ปรับตรง gyro 20 รอบ แล้วเข้ากึ่งกลางทางแยกด้วยเซนเซอร์ C[] แล้วเบรกหยุด (พิมพ์เล็ก/ใหญ่เหมือนกัน)
 *   'b' || 'B' : ปรับตรง gyro 20 รอบ วิ่งเข้ากึ่งกลาง (C[]) ก่อน แล้ววิ่งต่อจนเซนเซอร์ B[] เจอเส้นอีกครั้ง (ทั้ง G และ GB เช็ค B[] เหมือนกัน) แล้วสะบัดมอเตอร์สั้น ๆ แล้วหยุด
 *   's' || 'S' : เบรกสั้น ๆ ทันที ไม่มีโหมดรอ PID แบบ 'S' ของ (A) (พิมพ์เล็ก/ใหญ่เหมือนกัน)
 *                G:  ถอยสั้น ๆ (10→1) แล้วหยุด | GB: เดินหน้าสั้น ๆ (10→1) แล้วหยุด
 *   'a'/'A', 'd'/'D', 'g'/'G' : ⚠️ ไม่มี case รองรับในตระกูลนี้ ตกไปเงื่อนไข else เหมือนอักขระอื่น ๆ ที่ไม่รู้จัก → เรียก SetG(100) เสมอ
 *                                 (แม้แต่ TrackSelectGB ก็เรียก SetG(100) ไม่ใช่ SetGB — พฤติกรรมเหมือนกันทั้งสองฟังก์ชัน)
 */


/* =====================================================
   🚗 LINE FOLLOW PID : FORWARD (ff)
   ===================================================== */

// ทั่วไป
// ff(speed, 'คำสั่งทางแยก');
// ตัวอย่าง: ff(100, 'p');

// กลาง / ซ้าย / ขวา
// ffc(speed, 'คำสั่งทางแยก');     | ffc2(speed, 'คำสั่งทางแยก');
// ffl(speed, 'คำสั่งทางแยก');     | ffl2(speed, 'คำสั่งทางแยก'); | ffl0(speed, 'คำสั่งทางแยก');
// ffr(speed, 'คำสั่งทางแยก');     | ffr2(speed, 'คำสั่งทางแยก'); | ffr7(speed, 'คำสั่งทางแยก');

// ระบุเซนเซอร์ / ระยะ
// ffnum(speed, 'คำสั่งทางแยก', 0-7);
// ff_distance(speed, 'คำสั่งทางแยก', dist);
// ffd(speed, 'คำสั่งทางแยก', dist);
// ffds(speed, 'คำสั่งทางแยก', ir_target);        // = ff_distances(...) เหมือน ffd แต่ค่อยๆ เบรกเข้าเป้าหมาย (ลดความเร็วด้วย PID ตามระยะ)

// เจอ / ไม่เจอเส้น
// ffwhite(speed, 'คำสั่งทางแยก');
// ffblack(speed, 'คำสั่งทางแยก');
// ffw(speed, 'คำสั่งทางแยก');
// ffb(speed, 'คำสั่งทางแยก');


/* =====================================================
   🔙 LINE FOLLOW PID : BACKWARD (bb)
   ===================================================== */

// ทั่วไป
// bb(speed, 'คำสั่งทางแยก');

// กลาง / ซ้าย / ขวา
// bbc(speed, 'คำสั่งทางแยก');     | bbc2(speed, 'คำสั่งทางแยก');
// bbl(speed, 'คำสั่งทางแยก');     | bbl2(speed, 'คำสั่งทางแยก'); | bbl0(speed, 'คำสั่งทางแยก');
// bbr(speed, 'คำสั่งทางแยก');     | bbr2(speed, 'คำสั่งทางแยก'); | bbr7(speed, 'คำสั่งทางแยก');

// ระบุเซนเซอร์ / ระยะ
// bbnum(speed, 'คำสั่งทางแยก', 0-7);
// bb_distance(speed, 'คำสั่งทางแยก', dist);
// bbd(speed, 'คำสั่งทางแยก', dist);
// (ไม่มี bb_distances/bbds — ฟังก์ชันเบรกนุ่มนวลมีเฉพาะฝั่งหน้า)

// เจอ / ไม่เจอเส้น
// bbwhite(speed, 'คำสั่งทางแยก');
// bbblack(speed, 'คำสั่งทางแยก');
// bbw(speed, 'คำสั่งทางแยก');
// bbb(speed, 'คำสั่งทางแยก');


/* =====================================================
   ⏱️ LINE FOLLOW PID : TIMER / DISTANCE
   ===================================================== */

// ไม่มีคำสั่ง
// fftimer(speed, time);
// bbtimer(speed, time);

// มีคำสั่ง
// fftimer(speed, time, 'คำสั่งทางแยก');    | fft(speed, time, 'คำสั่งทางแยก');    // fft = alias ของ fftimer (ต้องมี select เสมอ)
// bbtimer(speed, time, 'คำสั่งทางแยก');    | bbt(speed, time, 'คำสั่งทางแยก');    // bbt = alias ของ bbtimer (ต้องมี select เสมอ)

// ระยะทาง (ประมาณหน่วย ซม.)
// ffcm(speed, cm);
// bbcm(speed, cm);
// ffcm(speed, cm, 'คำสั่งทางแยก');
// bbcm(speed, cm, 'คำสั่งทางแยก');


/* =====================================================
   🎯 CENTER ALIGNMENT (เข้ากึ่งกลางทางแยก)
   ===================================================== */

// เดินหน้า
// ToCenter();     // เจอเส้นข้างซ้ายหรือขวาก็ได้
// ToCenterL();    // เจอเส้นข้างซ้าย
// ToCenterR();    // เจอเส้นข้างขวา
// ToFront();      // วิ่ง PID ไปข้างหน้าจนเซนเซอร์หน้าเจอขอบเส้น F[0]/F[7] (ใช้ tctL/tctR)

// ถอยหลัง
// BackCenter();   // เจอเส้นข้างซ้ายหรือขวาก็ได้
// ToBack();       // วิ่ง PID ถอยหลังจนเซนเซอร์หลังเจอขอบเส้น B[0]/B[7] (ใช้ tctL/tctR)

// จัดตำแหน่งกึ่งกลางเส้นแบบละเอียด (ตอนวางหุ่นก่อนเริ่ม)
// set_f(num);   set_fc(num);      // ด้านหน้า (เซนเซอร์ล้อ / เซนเซอร์ข้าง)
// set_b(num);   set_bc(num);      // ด้านหลัง
// balancef(num) / setf(num);      // จัดหุ่นให้ตรงเส้นด้านหน้าด้วยเซนเซอร์ล้อ F[] ทำซ้ำ num รอบ
// balanceb(num) / setb(num);      // จัดหุ่นให้ตรงเส้นด้านหลังด้วยเซนเซอร์ล้อ B[] ทำซ้ำ num รอบ
// balancefc(num) / setfc(num);    // จัดหุ่นให้ตรงเส้นด้านหน้าด้วยเซนเซอร์คู่กลาง C[] ทำซ้ำ num รอบ
// balancebc(num) / setbc(num);    // จัดหุ่นให้ตรงเส้นด้านหลังด้วยเซนเซอร์คู่กลาง C[] ทำซ้ำ num รอบ

// ▶ ออกตัว / เข้าเส้นชัย
// gostart(speed);              gostart(LeftSpeed, RightSpeed);   // เดินหน้าออกจากเส้นเริ่มต้น จนเซนเซอร์หน้าหลุดเส้น (F[0]/F[7] ไม่เจอเส้นแล้ว)
// goend(speed);                goend(LeftSpeed, RightSpeed);     // เดินหน้าจนเซนเซอร์กลางทั้งคู่เจอเส้น (เข้าเส้นชัย) แล้วเบรกหยุด

// ▶ เรียกจัดการ "คำสั่งทางแยก" ตรง ๆ (ปกติ ff/bb ทุกตัวเรียกให้อัตโนมัติเมื่อเจอทางแยกอยู่แล้ว
//    ใช้เองเฉพาะกรณีเช็คเงื่อนไขเจอทางแยกเองแล้ว อยากสั่งเลี้ยว/หยุดต่อทันที)
// TrackSelectF(speed, 'คำสั่งทางแยก');    // เดินหน้า (ตัวที่ ff/ffc/ffl/ffr ฯลฯ เรียกใช้ภายใน)
// TrackSelectB(speed, 'คำสั่งทางแยก');    // ถอยหลัง (ตัวที่ bb/bbc/bbl/bbr ฯลฯ เรียกใช้ภายใน)


/* =====================================================
   🔄 เลี้ยว / หมุน ด้วยเซนเซอร์เส้น
   ===================================================== */

// SpinL / SpinR = หมุนอยู่กับที่ 90°
// spinl(speed);    spinr(speed);          // ไม่ใส่ speed = ใช้ค่าจาก SetTurnSpeed()
// spinl2(speed);   spinr2(speed);         // เดินเลยเส้นก่อนแล้วค่อยหมุน แม่นกว่า

// TurnL / TurnR = เลี้ยวโค้ง (ล้อเดียวหมุน) ใช้ TurnSpeedLeft()/TurnSpeedRight() ที่ตั้งไว้
// TurnLeft();      TurnRight();

// เลี้ยวล้อเดียวแบบถอยหลัง เช็คเส้นด้วยเซนเซอร์หน้า F[] ใช้ TurnSpeedLeftBackF()/TurnSpeedRightBackF() ที่ตั้งไว้
// TurnLeftBackF();  TurnRightBackF();

// ชุดเดียวกันแต่ใช้เซนเซอร์ฝั่งหลัง (ตอนถอยหลัง)
// spinl_B(speed);  spinr_B(speed);
// spinl2_B(speed); spinr2_B(speed);
// TurnLeft_B();    TurnRight_B();

// เลี้ยวล้อเดียวแบบถอยหลัง เช็คเส้นด้วยเซนเซอร์หลัง B[] ใช้ TurnSpeedLeftBackB()/TurnSpeedRightBackB() ที่ตั้งไว้
// TurnLeftBackB();  TurnRightBackB();


/* =====================================================
   📡 IMU / GYROSCOPE COMMAND REFERENCE
   ===================================================== */

// 🔧 การตั้งค่าเริ่มต้น
// resetAngles();        // รีเซ็ตมุมอ้างอิงของ IMU (ทำตอนหุ่นอยู่นิ่ง)
// SetRobotAngle();       // อ่านมุมปัจจุบันเก็บเป็นค่าอ้างอิง (current_degree)
// gyroZ();                // คืนค่ามุมปัจจุบัน (องศา)

// 🔒 ล็อกมุมอยู่กับที่ (กันหุ่นเบี้ยว)
// SetHoldAngle();
// SetFG(ms);   setfg(ms);      // โหมดหน้า
// SetG(ms);    setg(ms);       // โหมดทั่วไป
// SetGB(ms);   setgb(ms);      // โหมดถอยหลัง

// SetFG/SetG/SetGB ข้างบนคือ loop เรียก HoldAngleF()/HoldAngle()/HoldAngleB() ซ้ำจนครบเวลาที่กำหนด
// เรียกฟังก์ชันเดี่ยว ๆ ด้านล่างเองได้ ถ้าต้องการควบคุม loop เอง (เช่นเรียกซ้ำใน loop() หลักของสเก็ตช์)
// HoldAngle();     // ล็อกมุมไว้ที่ current_degree (ตั้งค่าไว้ล่าสุดจาก SetRobotAngle())
// HoldAngleB();    // เหมือน HoldAngle() แต่กลับทิศมอเตอร์ (ใช้ตอนถอยหลัง)
// HoldAngleF();    // ล็อกมุมไว้ที่ holdAngle (ตั้งค่าไว้ล่าสุดจาก SetHoldAngle())
// ปรับความไวได้ด้วยตัวแปร kpHold/kdHold, kpBHold/kdBHold, kpFHold/kdFHold

// 🔄 หมุน / เลี้ยว ตามองศาที่กำหนด (ความเร็วอัตโนมัติ)
// spindegree(Angle);          turndegree(Angle);          turndegreeb(Angle);
// ตัวอย่าง: spindegree(-90);   // หมุนซ้าย 90°   (ลบ = ซ้าย, บวก = ขวา)

// ⚙️ แบบกำหนดความเร็วเอง
// spindegree(speed, Angle);   turndegree(speed, Angle);   turndegreeb(speed, Angle);
// ตัวอย่าง: turndegree(50, 90);   // เลี้ยวขวา ความเร็ว 50

// ชื่อย่อ (ใช้แข่ง เขียนเร็ว) — ความหมายเหมือนกันทุกตัว มี/ไม่มี speed ได้ทั้งคู่
// spinlg / spinrg      = spindegree ซ้าย/ขวา
// turnlg / turnrg      = turndegree ซ้าย/ขวา
// turnlbg / turnrbg    = turndegreeb ซ้าย/ขวา
// slg/srg/tlg/trg/tlbg/trbg   = ชื่อสั้นอีกชุดของ 6 ตัวข้างบน ความหมายเดียวกัน

// 🔁 หมุน "ไม่เบรก" (สำหรับต่อท่าซ้อนกัน เช่น หมุนซ้ายแล้วหมุนขวาทันที)
// turndegree_none(Angle);            turndegree_none(speed, Angle);       // ค่า default speed = 50 ถ้าไม่ระบุ
// turndegreeb_none(Angle);           turndegreeb_none(speed, Angle);      // เหมือนกันแต่ตอนถอยหลัง
// ต่างจาก turndegree/turndegreeb ตรงที่ไม่ MotorStop() ตอนจบ (ไหลต่อท่าถัดไปได้ลื่นกว่า)

// 🔀 หมุนสองจังหวะต่อเนื่อง (ซ้ายแล้วขวา หรือขวาแล้วซ้าย) ไม่เบรกกลาง — ใช้ turndegree_none/turndegreeb_none ข้างในแล้วปิดท้ายด้วย SetG
// tlrg(Angle);              trlg(Angle);              // เดินหน้า: ซ้ายแล้วขวา (tlrg) / ขวาแล้วซ้าย (trlg) มุมเท่ากันทั้งสองจังหวะ
// tlrg(speed, Angle);       trlg(speed, Angle);        // กำหนดความเร็วเอง มุมเท่ากันทั้งสองจังหวะ
// tlrg(speed, Angle, Angle2);  trlg(speed, Angle, Angle2);  // กำหนดมุมจังหวะแรก/สองแยกกัน (ไม่มี SetG ปิดท้าย)
// tlrbg(Angle);             trlbg(Angle);              // เหมือนชุด tlrg/trlg แต่เป็นเวอร์ชันถอยหลัง (ใช้ turndegreeb_none)
// tlrbg(speed, Angle);      trlbg(speed, Angle);
// tlrbg(speed, Angle, Angle2); trlbg(speed, Angle, Angle2);

// 🌙 หมุนโค้งแบบวงกว้าง (arc: ล้อสองข้างวิ่งพร้อมกันคนละความเร็ว ไม่ใช่หมุนอยู่กับที่)
// rotatedegree(SpeedL, SpeedR, relative_degree);              // ใช้ kp/kd default (0.9, 0.6)
// rotatedegree(SpeedL, SpeedR, relative_degree, kp, kd);       // กำหนด kp/kd เอง
// ตัวอย่าง: rotatedegree(60, 20, 90);   // โค้งขวา 90° ล้อซ้ายเร็วกว่าล้อขวา


/* =====================================================
   🚀 การวิ่งตรง / ตามระยะ ด้วย Gyro (RunG แทน PIDF)
   ===================================================== */

// ▶ วิ่งตามเวลา
// fftimerg(speed, time);                    bbtimerg(speed, time);
// fftimerg(speed, time, 'คำสั่งทางแยก');      bbtimerg(speed, time, 'คำสั่งทางแยก');
// fftg(speed, time, 'คำสั่งทางแยก');          bbtg(speed, time, 'คำสั่งทางแยก');

// ▶ วิ่งตามระยะ (ซม.) — มีเร่ง/ผ่อนความเร็วอัตโนมัติช่วงต้น-ท้าย
// ffcmg(speed, cm , 'คำสั่งทางแยก');        bbcmg(speed, cm , 'คำสั่งทางแยก');

// ▶ วิ่งตามระยะ ความเร็วคงที่ตลอด (ไม่เร่ง/ผ่อน)
// ffcmgs(speed, cm [, 'คำสั่งทางแยก']);       bbcmgs(speed, cm , 'คำสั่งทางแยก');

// ▶ วิ่งจนเจอเส้นดำตรงกลาง (คุมทิศด้วย gyro)
// ffbg(speed, 'คำสั่งทางแยก');                bbbg(speed, 'คำสั่งทางแยก');

// ▶ วิ่งจนถึงระยะจากเซนเซอร์วัดระยะ
// ffdg(speed, 'คำสั่งทางแยก', dist);          bbdg(speed, 'คำสั่งทางแยก', dist);
// ffdgs(speed, 'คำสั่งทางแยก', dist);         bbdgs(speed, 'คำสั่งทางแยก', dist);

// ▶ เข้ากึ่งกลางทางแยกด้วย gyro
// ToCenterLG();   ToCenterRG();   ToCenterLRG();   BackCenterG();

// ▶ เรียกจัดการ "คำสั่งทางแยก" ตรง ๆ ด้วย gyro (ปกติ ffbg/bbbg เรียกให้อัตโนมัติเมื่อเจอเส้นอยู่แล้ว
//    ใช้ 2 ตัวนี้เองเฉพาะกรณีเช็คเงื่อนไขเจอทางแยกเองแล้ว อยากสั่งเลี้ยว/หยุดต่อทันที)
// TrackSelectG(speed, 'คำสั่งทางแยก');    // เดินหน้า
// TrackSelectGB(speed, 'คำสั่งทางแยก');   // ถอยหลัง


/* =====================================================
   🌀 Circle Motion Functions (วิ่งเบี่ยงซ้าย/ขวา)
   เปลี่ยนค่า set_position ชั่วคราวเป็น set_positionL/R เพื่อวิ่งโค้ง
   =====================================================
   ┌────────────────────────────────────────────────────────────────┐
   │ Function                          Example                      │
   ├────────────────────────────────────────────────────────────────┤
   │ ffcl(Speed, select)               ffcl(60, 'p');                │
   │ ffcr(Speed, select)               ffcr(60, 'p');                │
   │--------------------------------------------------------------- │
   │ fftimercl(Speed, time)            fftimercl(60, 500);           │
   │ fftimercr(Speed, time)            fftimercr(60, 500);           │
   │--------------------------------------------------------------- │
   │ ffcmcl(Speed, distance)           ffcmcl(60, 20);                │
   │ ffcmcr(Speed, distance)           ffcmcr(60, 20);                │
   │--------------------------------------------------------------- │
   │ fftimercl(Speed, time, select)    fftimercl(60, 500, 'p');       │
   │ fftimercr(Speed, time, select)    fftimercr(60, 500, 'p');       │
   │--------------------------------------------------------------- │
   │ ffcmcl(Speed, distance, select)   ffcmcl(60, 20, 'p');           │
   │ ffcmcr(Speed, distance, select)   ffcmcr(60, 20, 'p');           │
   ├────────────────────────────────────────────────────────────────┤
   │ bbcl(Speed, select)               bbcl(60, 'p');                │
   │ bbcr(Speed, select)               bbcr(60, 'p');                │
   │--------------------------------------------------------------- │
   │ bbtimercl(Speed, time)            bbtimercl(60, 500);           │
   │ bbtimercr(Speed, time)            bbtimercr(60, 500);           │
   │--------------------------------------------------------------- │
   │ bbcmcl(Speed, distance)           bbcmcl(60, 20);                │
   │ bbcmcr(Speed, distance)           bbcmcr(60, 20);                │
   │--------------------------------------------------------------- │
   │ bbtimercl(Speed, time, select)    bbtimercl(60, 500, 'p');       │
   │ bbtimercr(Speed, time, select)    bbtimercr(60, 500, 'p');       │
   │--------------------------------------------------------------- │
   │ bbcmcl(Speed, distance, select)   bbcmcl(60, 20, 'p');           │
   │ bbcmcr(Speed, distance, select)   bbcmcr(60, 20, 'p');           │
   └────────────────────────────────────────────────────────────────┘

   Parameters
     Speed    : ความเร็วมอเตอร์
     time     : เวลา (ms)
     distance : ระยะทาง (ซม. โดยประมาณ)
     select   : คำสั่งทางแยก (ดูตารางหมวด 7️⃣)

   Notes
     - CL = Circle Left, CR = Circle Right
     - ต้องตั้ง set_position_line_l()/set_position_line_r() ไว้ก่อน (ใน Setting.ino)
   ===================================================== */


/* =====================================================
   🖐️ ARM & SERVO COMMANDS
   ===================================================== */

// ระดับไลบรารี (Servo.h)
// armupdown(องศา);                    armupdown(องศา, ความเร็ว);
// arm_left_right(องศาซ้าย, องศาขวา);   arm_left_right(องศาซ้าย, องศาขวา, ความเร็ว);
// Servo(ขา, องศา);                     // ขา: 0, 1, 10, 28
// Servo(x, y, z);                      Servo(t1, t2, t3, speed);
// S0_trim(x); S1_trim(x); S10_trim(x); S28_trim(x);   // ปรับ trim แต่ละขา
// SerialServoControl();                // ตั้งค่าเซอร์โวผ่าน Serial (พิมพ์ "ขา องศา" เช่น "10 90")

// ระดับสเก็ตช์ (Servo.ino — แขนกล/มือจับ)
// arm_ready()        arm_open_down()     arm_down_open()
// arm_open_up()       arm_up_open()       arm_down_close()
// arm_close_down()    arm_up_close()      arm_close_up()
// arm_big_box()        arm_big_box_up()    arm_behihd()
// arm_up()             arm_up45()          arm_down()          arm_open()
// arm_close()          arm_big()           arm_open_l()      arm_open_r()
// (ทุกฟังก์ชันข้างบนมีรุ่น (spd) ต่อท้ายได้ เช่น arm_close(2); = หุบมือแบบไล่มุมนุ่มนวล)
// ค่าองศาที่ตั้งไว้: servo_down, servoL_open, servoR_open, readyL/readyR,
//                    behindL/behindR, up, up45, closeL/closeR, closeBigL/closeBigR


/* =====================================================
   👂 เซนเซอร์เส้น & คาลิเบรต
   ===================================================== */

// อ่านค่า
// ReadSensor();   ReadSensorRaw();
// ReadCalibrateF();   ReadCalibrateB();   ReadCalibrateC();

// คาลิเบรต
// calibrateA();   calibrateB();   calibrateC();                    // -> EEPROM ภายนอก
// saveCalibA_LOCAL();  saveCalibB_LOCAL();  saveCalibC_LOCAL();     // -> flash ในบอร์ด (ใช้จริง)
// loadCalibration();   loadCalibration_LOCAL();

// ตั้งค่าพฤติกรรม
// TrackLineColor(0/1);           // 0=พื้นขาวเส้นดำ, 1=พื้นดำเส้นขาว
// Dottedline(0/1);                // 1=สนามมีเส้นประ
// clampSensorValueF(min,max);   clampSensorValueB(min,max);   clampSensorValueC(min,max);
// RefLineValue(x);   RefCenterLineValue(x);   SetAnalogDistance(pin);


/* =====================================================
   ⚙️ มอเตอร์พื้นฐาน & การตั้งค่า PID
   ===================================================== */

// Motor(L, R);              Move(L, R, time_ms);
// MotorStop();               MotorStop(t);          // มี t = บี๊บด้วย
// MotorShot(t, power);       // เบรกกะทันหันแบบ active short-brake (ลัดวงจรผ่าน back-EMF); t=เวลาคงเบรก(ms, ค่าเริ่มต้น 3), power=แรงเบรก 0-100 (ค่าเริ่มต้น 90) แล้วปล่อยกลับ MotorStop() อัตโนมัติ

// เดินหน้า / ถอย / หมุน / เลี้ยวล้อเดียว ตามเวลา
// fd(speed, time_ms);   bk(speed, time_ms);
// sl(speed, time_ms);    sr(speed, time_ms);
// tl(speed, time_ms);    tr(speed, time_ms);

// ตารางชดเชย/PID ต่อช่วงความเร็ว (Setting.ino, ch = SPD_10 ... SPD_100)
// setBalanceSpeed(ch, l, r);        setBalanceBackSpeed(ch, l, r);
// Set_KP_KD(ch, kp, kd);            Set_KP_KD_Back(ch, kp, kd);

// ตั้งค่า PID/ทางแยกอื่น ๆ (Setting.ino)
// set_position_line(pos);   set_position_line_l(pos);   set_position_line_r(pos);
// set_line_center(0/1);      SetToCenterSpeed(speed);
// set_slow_kp_kd(kpf, kdf, kpb, kdb);
// SetTurnSpeed(spd);          TurnSpeedLeft(l, r, delay);   TurnSpeedRight(l, r, delay);
// ModeSpdPID(mode, max, min);
// SetPIDDeadBand(db);         // ค่า error ต่ำกว่านี้มองเป็น 0 กันสั่น (ค่าเริ่มต้น 20)


/* =====================================================
   🔔 เสียง
   ===================================================== */

// Beep(ms);   Beep2(freq, ms);   BZon();   BZoff();
// beep(freq, dur);   blink(times);
// BeepScanner();   // ปี๊บสั้นแหลมครั้งเดียว คล้ายเครื่องยิงบาร์โค้ด (= beep(3000, 80))


/* =====================================================
   ⚠️ ข้อควรระวัง
   =====================================================
 *  1) select ต้องใส่ในเครื่องหมายคำพูดเดี่ยวเสมอ เช่น ff(100, 'p'); ไม่ใช่ ff(100, p);
 *  2) ฟังก์ชัน ff, bb, ToCenter (ทุกตัวที่ขึ้นต้นด้วยชื่อเหล่านี้) ที่รอเงื่อนไขเส้นเป็น while(1) ไม่มี timeout
 *     ถ้าชนสิ่งกีดขวางจนเซนเซอร์ไม่เจอเงื่อนไข จะค้างสั่งมอเตอร์เต็มกำลังไม่มีที่สิ้นสุด
 *  3) ต้องคาลิเบรตเซนเซอร์ผ่าน sw() อย่างน้อย 1 ครั้งบนสนามจริงก่อนแข่งเสมอ
 *  4) ฟังก์ชัน Serial_, SerialCalibrate_, SerialPosition (ตระกูล debug) เป็น while(1) ห้ามเรียกใน Mission()
   ===================================================== */
