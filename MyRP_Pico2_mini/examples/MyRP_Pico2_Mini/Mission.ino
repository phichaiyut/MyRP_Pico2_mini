void Mission() {
  // ===== Line Tracking Setup =====

  // Center mode
  // set_line_center(0);  // เดินธรรมดา เข้ากลางหุ่น
  set_line_center(1);  // เดินตามเส้น เข้ากลางหุ่น

  // Line color
  TrackLineColor(0);  // 0 = พื้นขาวเส้นดำ | 1 = พื้นดำเส้นขาว

  // Dotted line
  Dottedline(0);  // 0 = ไม่มีเส้นประ | 1 = มีเส้นประ

  // PID speed mode
  // ModeSpdPID(0, 100, -5);  // โหมดใช้งานปัจจุบัน

  // ===== PID Mode Examples =====
  ModeSpdPID(1, 100, -100);  // PID แบบปกติ
  // ModeSpdPID(2, 60, -60);    // โค้งเยอะ
  // ModeSpdPID(3, 100, -50);   // โหมดคิดเอง
  // ModeSpdPID(4, 100, 0);     // PID แบบปกติ

}//______________ปีกกาห้ามลบ_____________