// 셈틀
//
// 배선과 게이트만으로 회로를 조합하는 판.
// 이름은 '컴퓨터' 를 순우리말로 쓰던 말에서 왔다 — 셈(계산) + 틀(기계).
// 부품(스위치·전구·게이트)을 끌어다 놓고, 포트끼리 선으로 잇는다.
// 매 틱마다 게이트가 입력을 보고 출력을 정한다 — 한 틱 지연이 있어서
// 되먹임으로 물리면 기억(래치)도 생긴다.
//
// 만든 회로를 상자로 묶어 커스텀 게이트로 쓸 수 있다. 안의 스위치가
// 입력 포트, 전구가 출력 포트가 된다. 상자 안에 상자도 된다(중첩).
//
// 빌드: make
// 실행: ./semtle
// 검사: ./semtle --test  (화면 없이 회로 논리만 돌린다)

#include <SDL2/SDL.h>

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// ─────────────────────────────────────────────────────────────
// 크기
// ─────────────────────────────────────────────────────────────

// 글자와 판이 전부 이 배로 커진다. 화면마다 알맞은 크기가 달라서 손으로 바꿀 수 있게 뒀다.
// 아래 자리·크기 숫자는 다 '1배일 때' 값이고, 쓸 때 S() 로 늘린다.
static float uiScale = 1.35f;
static const float UI_MIN = 0.8f, UI_MAX = 2.2f;

static inline int S(int v) { return (int)(v * uiScale + 0.5f); }

static const int PANEL_W0 = 190;      // 왼쪽 도구판 (1배일 때)
static const int LES_W0   = 268;      // 오른쪽 설명판 (1배일 때)
static int PANEL_W = PANEL_W0;
static int LES_W   = LES_W0;

// 창 크기는 사람이 바꿀 수 있다. 왼쪽 도구판과 오른쪽 설명판은 너비가 그대로고,
// 늘어나고 줄어드는 몫은 가운데 판이 다 가져간다.
static int WIN_W = 0, WIN_H = 0;
static int WIN_W0 = 0, WIN_H0 = 0;          // 처음 크기
static int WIN_W_MIN = 0, WIN_H_MIN = 0;

// 배율이 바뀌면 판 너비와 최소 창 크기를 다시 잡는다
static void applyUiScale() {
    uiScale  = std::clamp(uiScale, UI_MIN, UI_MAX);
    PANEL_W  = S(PANEL_W0);
    LES_W    = S(LES_W0);
    WIN_W0   = PANEL_W + S(940);
    WIN_H0   = S(720);
    WIN_W_MIN = PANEL_W + LES_W + S(340);
    WIN_H_MIN = S(560);
    if (WIN_W < WIN_W_MIN) WIN_W = WIN_W_MIN;
    if (WIN_H < WIN_H_MIN) WIN_H = WIN_H_MIN;
}

static const int GW_MIN = 74;     // 부품 상자 최소 가로
static const int PORT_R = 6;      // 포트 반지름(그리기)
static const int PORT_HIT = 11;   // 포트 잡히는 반경

// ─────────────────────────────────────────────────────────────
// 붙박이 부품
// ─────────────────────────────────────────────────────────────

enum Type { SWITCH, LAMP, PIN_IN, PIN_OUT,
            T_AND, T_OR, T_NOT, T_XOR, T_NAND, T_NOR,
            CLOCK, BUNDLE, SPLIT, RAM, SWBANK, SCREEN, TYPE_N };

struct TypeInfo { const char* name; int nIn; int nOut; uint32_t color; };

static const TypeInfo TYPES[TYPE_N] = {
    /* SWITCH  */ { "스위치", 0, 1, 0x2E7D5B },
    /* LAMP    */ { "전구",   1, 0, 0x8A6D3B },
    /* PIN_IN  */ { "입력핀", 0, 1, 0x3E8ACC },   // 칩의 입력 포트가 된다
    /* PIN_OUT */ { "출력핀", 1, 0, 0xCC7A3E },   // 칩의 출력 포트가 된다
    /* AND     */ { "AND",    2, 1, 0x35688A },
    /* OR      */ { "OR",     2, 1, 0x35688A },
    /* NOT     */ { "NOT",    1, 1, 0x9A4A5A },
    /* XOR     */ { "XOR",    2, 1, 0x6A5AA0 },
    /* NAND    */ { "NAND",   2, 1, 0x4A6A44 },
    /* NOR     */ { "NOR",    2, 1, 0x4A6A44 },
    /* CLOCK   */ { "클럭",   0, 1, 0xC8963C },   // 박자를 준다
    /* BUNDLE  */ { "묶음",   2, 1, 0x4E7E7E },   // 가는 선 여럿 → 굵은 선 하나
    /* SPLIT   */ { "풀음",   1, 2, 0x4E7E7E },   // 굵은 선 하나 → 가는 선 여럿
    /* RAM     */ { "램",     4, 1, 0x8A4E7E },   // 주소·데이터·쓰기·클럭 → 데이터
    /* SWBANK  */ { "스위치묶음", 0, 1, 0x2E7D5B }, // 스위치 n개를 한 값으로
    /* SCREEN  */ { "화면",   1, 0, 0x3E5A6E },   // n개의 m비트를 점 격자로
};

// 칩의 포트가 되는 부품
static inline bool isPin(int type) { return type == PIN_IN || type == PIN_OUT; }
// 폭을 사람이 정할 수 있는 부품 (aux 에 폭이 들어간다)
static inline bool hasWidth(int type) {
    return isPin(type) || type == BUNDLE || type == SPLIT || type == SWBANK || type == SCREEN;
}

// 화면 크기: aux 아랫자리는 비트 폭(m), 그 위 자리는 줄 수(n)
static const int SCR_ROWS_SHIFT = 12;      // aux 의 이 자리부터 줄 수
static const int SCR_MAX = 16;

// 램 크기
static const int RAM_WORDS = 256;
static const int RAM_ADDRW = 8;
static const int RAM_DATAW = 8;

// 클럭이 몇 틱마다 뒤집힐지 (aux 가 이 표의 자리)
static const int CLOCK_TICKS[] = { 1, 2, 4, 8, 16, 32, 64 };
static const int CLOCK_N = 7;

// ─────────────────────────────────────────────────────────────
// 회로 자료구조
//
// SubSim 은 부품과 선의 묶음이다. 판 전체(world)도 SubSim 이고,
// 커스텀 게이트 안쪽도 SubSim 이다 — 같은 코드로 돌린다.
// ─────────────────────────────────────────────────────────────

struct SubSim;   // 앞선언

// 신호는 이제 0/1 이 아니라 폭만큼의 수다. 폭 1이면 예전과 똑같이 0 아니면 1.
// 8비트 선 하나가 예전의 선 여덟 가닥을 대신한다.
typedef uint16_t Val;
static const int WIDTH_MAX = 16;

static inline Val maskTo(Val v, int w) {
    if (w >= 16) return v;
    return (Val)(v & ((1u << w) - 1));
}

struct Comp {
    int  type = -1;           // 붙박이면 Type, 커스텀 게이트면 -1 (chipId 로 구분)
    int  chipId = -1;         // chips[] 자리 (커스텀 게이트일 때)
    int  x = 0, y = 0;
    int  rot = 0;             // 0 = 출력이 오른쪽, 1 = 아래, 2 = 왼쪽, 3 = 위
    int  aux = 0;             // 부품마다 뜻이 다른 값 (클럭 주기, 핀·묶음 폭 …)
    int  tickAcc = 0;         // 클럭이 센 틱
    uint8_t lastClk = 0;      // 램이 본 지난 클럭 (올라가는 순간을 잡으려고)
    std::string label;        // 핀 이름 (핀이 아니면 빈 칸)
    std::vector<Val>     in;       // 이번 틱 입력값들
    std::vector<Val>     out;      // 지금 출력값들
    std::vector<Val>     nextOut;  // 다음 틱에 반영될 값
    std::vector<uint8_t> inW, outW;   // 포트마다의 폭 (1~16)
    std::vector<Val>     mem;      // 램의 속 (램이 아니면 비어 있다)
    bool alive = true;
    std::shared_ptr<SubSim> sub;   // 커스텀 게이트의 속 회로 (인스턴스마다 따로)
};

struct Wire { int from, fromPort, to, toPort; bool alive; };

struct SubSim {
    std::vector<Comp> comps;
    std::vector<Wire> wires;
    std::vector<int>  inPorts;    // 입력핀들의 자리 (위→아래 순)
    std::vector<int>  outPorts;   // 출력핀들의 자리 (위→아래 순)
};

// 지운 칩은 자리를 비우지 않고 alive 만 끈다. chipId 가 chips[] 의 자리라서,
// 진짜로 빼면 뒤엣것들의 번호가 다 밀려 이미 놓인 상자들이 엉뚱한 걸 가리킨다.
struct Chip { std::string name; uint32_t color; SubSim tmpl; bool alive = true; };

static SubSim world;
static std::vector<Chip> chips;

// 목록에 보이는 칩들 (지운 것은 뺀다)
static std::vector<int> liveChips() {
    std::vector<int> v;
    for (int i = 0; i < (int)chips.size(); ++i) if (chips[i].alive) v.push_back(i);
    return v;
}

static const uint32_t CHIP_COLORS[] = {
    0x4C7A9E, 0x9E7A4C, 0x6E9E4C, 0x9E4C7A, 0x4C9E8E, 0x7A4C9E, 0x9E5A4C, 0x5A5A9E,
};

// aux 는 폭(아랫자리)과 접힘 표시(0x100)를 같이 들고 있다.
static const int FOLD_BIT = 0x100;

// 폭을 정할 수 있는 부품의 폭 (aux). 없으면 1.
static int widthOf(const Comp& c) {
    if (!hasWidth(c.type) || c.chipId >= 0) return 1;
    int w = c.aux & 0xFF;
    return std::clamp(w <= 0 ? 1 : w, 1, WIDTH_MAX);
}
// 접을 수 있는 것은 묶음·풀음뿐 (가닥이 여럿이라 자리를 많이 먹는다)
static bool canFold(const Comp& c) {
    return c.chipId < 0 && (c.type == BUNDLE || c.type == SPLIT) && widthOf(c) > 2;
}
static bool isFolded(const Comp& c) {
    return canFold(c) && (c.aux & FOLD_BIT);
}
// 칸을 눌러 비트를 켜는 것들. 폭 있는 입력핀도 스위치묶음처럼 다룬다 —
// 안 그러면 8비트 핀을 눌러도 0 아니면 1 밖에 못 넣는다.
static bool isBitBank(const Comp& c) {
    if (c.chipId >= 0) return false;
    if (c.type == SWBANK) return true;
    return c.type == PIN_IN && widthOf(c) > 1;
}
// 화면이 받는 줄 수 (n). 폭(m)은 widthOf 가 준다.
static int screenRows(const Comp& c) {
    if (c.type != SCREEN) return 1;
    int r = (c.aux >> SCR_ROWS_SHIFT) & 0x1F;
    return std::clamp(r <= 0 ? 8 : r, 1, SCR_MAX);
}

// 입력·출력 포트 수
static int nIn(const Comp& c)  {
    if (c.chipId >= 0) return (int)chips[c.chipId].tmpl.inPorts.size();
    if (c.type == BUNDLE) return widthOf(c);      // 가는 선 폭만큼 받는다
    if (c.type == SPLIT)  return 1;
    if (c.type == SCREEN) return screenRows(c);   // 줄마다 하나씩 받는다
    return TYPES[c.type].nIn;
}
static int nOut(const Comp& c) {
    if (c.chipId >= 0) return (int)chips[c.chipId].tmpl.outPorts.size();
    if (c.type == BUNDLE) return 1;
    if (c.type == SPLIT)  return widthOf(c);      // 폭만큼 내보낸다
    return TYPES[c.type].nOut;
}

// 포트 하나의 폭
static int inWidth(const Comp& c, int port) {
    if (c.chipId >= 0) {
        const SubSim& t = chips[c.chipId].tmpl;
        if (port >= 0 && port < (int)t.inPorts.size()) {
            int i = t.inPorts[port];
            if (i >= 0 && i < (int)t.comps.size()) return widthOf(t.comps[i]);
        }
        return 1;
    }
    if (c.type == PIN_OUT) return widthOf(c);
    if (c.type == SPLIT)   return widthOf(c);     // 굵은 선을 받는다
    if (c.type == BUNDLE)  return 1;              // 가는 선들을 받는다
    if (c.type == SCREEN)  return widthOf(c);     // 줄 하나가 m비트
    if (c.type == RAM)     return (port == 0) ? RAM_ADDRW : (port == 1) ? RAM_DATAW : 1;
    return 1;
}
static int outWidth(const Comp& c, int port) {
    if (c.chipId >= 0) {
        const SubSim& t = chips[c.chipId].tmpl;
        if (port >= 0 && port < (int)t.outPorts.size()) {
            int i = t.outPorts[port];
            if (i >= 0 && i < (int)t.comps.size()) return widthOf(t.comps[i]);
        }
        return 1;
    }
    if (c.type == PIN_IN)  return widthOf(c);
    if (c.type == SWBANK)  return widthOf(c);     // 스위치 n개를 한 값으로
    if (c.type == BUNDLE)  return widthOf(c);     // 굵은 선을 내보낸다
    if (c.type == SPLIT)   return 1;
    if (c.type == RAM)     return RAM_DATAW;
    return 1;
}

// 포트 크기가 바뀌었을 때 값 그릇을 다시 맞춘다
static void resizePorts(Comp& c) {
    int ni = nIn(c), no = nOut(c);
    c.in.assign(ni, 0);
    c.out.resize(no, 0); c.out.resize(no);
    c.nextOut.assign(no, 0);
    c.inW.resize(ni); c.outW.resize(no);
    for (int i = 0; i < ni; ++i) c.inW[i]  = (uint8_t)inWidth(c, i);
    for (int i = 0; i < no; ++i) c.outW[i] = (uint8_t)outWidth(c, i);
    for (int i = 0; i < no; ++i) c.out[i] = maskTo(c.out[i], c.outW[i]);
    if (c.type == RAM && (int)c.mem.size() != RAM_WORDS) c.mem.assign(RAM_WORDS, 0);
}
static const char* compName(const Comp& c) {
    if (c.chipId >= 0) return chips[c.chipId].name.c_str();
    if (isPin(c.type) && !c.label.empty()) return c.label.c_str();   // 핀은 제 이름으로 보인다
    return TYPES[c.type].name;
}

// 칩의 포트 이름. 설계도 안의 핀에 붙여 둔 이름을 그대로 쓴다.
static const char* portName(const Comp& c, int port, bool isInput) {
    if (c.chipId < 0) return nullptr;
    const SubSim& t = chips[c.chipId].tmpl;
    const std::vector<int>& v = isInput ? t.inPorts : t.outPorts;
    if (port < 0 || port >= (int)v.size()) return nullptr;
    int i = v[port];
    if (i < 0 || i >= (int)t.comps.size()) return nullptr;
    return t.comps[i].label.empty() ? nullptr : t.comps[i].label.c_str();
}
static uint32_t compColor(const Comp& c) {
    return c.chipId >= 0 ? chips[c.chipId].color : TYPES[c.type].color;
}

// ─────────────────────────────────────────────────────────────
// 만들기 / 잇기 / 지우기
// ─────────────────────────────────────────────────────────────

static int addComp(SubSim& s, int type, int x, int y) {
    Comp c; c.type = type; c.chipId = -1; c.x = x; c.y = y;
    if (hasWidth(type)) {
        if (type == BUNDLE || type == SPLIT || type == SWBANK) c.aux = 8;
        else if (type == SCREEN) c.aux = 8 | (8 << SCR_ROWS_SHIFT);   // 8줄 × 8칸
        else c.aux = 1;
    }
    resizePorts(c);
    // 핀은 이름이 있어야 뜻이 있다. 없으면 안 겹치게 번호를 붙여 준다.
    if (isPin(type)) {
        int n = 0;
        for (const auto& o : s.comps) if (o.alive && o.type == type) ++n;
        char b[24]; std::snprintf(b, sizeof(b), "%s%d", type == PIN_IN ? "입력" : "출력", n + 1);
        c.label = b;
    }
    s.comps.push_back(std::move(c));
    return (int)s.comps.size() - 1;
}
static SubSim deepCopy(const SubSim& s);   // 앞선언

static int addChip(SubSim& s, int chipId, int x, int y) {
    // 없는 칩이면 아무것도 안 놓는다. -1 을 받은 addWire 는 그냥 넘어가니
    // 부르는 쪽이 매번 확인 안 해도 터지진 않는다.
    if (chipId < 0 || chipId >= (int)chips.size() || !chips[chipId].alive) return -1;
    Comp c; c.type = -1; c.chipId = chipId; c.x = x; c.y = y;
    c.sub = std::make_shared<SubSim>(deepCopy(chips[chipId].tmpl));
    resizePorts(c);
    s.comps.push_back(std::move(c));
    return (int)s.comps.size() - 1;
}

static SubSim deepCopy(const SubSim& s) {
    SubSim r;
    r.inPorts = s.inPorts; r.outPorts = s.outPorts; r.wires = s.wires;
    r.comps.reserve(s.comps.size());
    for (const auto& c : s.comps) {
        Comp n = c;                                   // 벡터는 복사, sub 는 얕게
        if (c.chipId >= 0 && c.sub)
            n.sub = std::make_shared<SubSim>(deepCopy(*c.sub));   // 속은 깊게
        r.comps.push_back(std::move(n));
    }
    return r;
}

// 입력 포트 하나엔 선 하나. 이미 있으면 끊고 갈아탄다.
// 폭이 다른 포트는 못 잇는다 (굵은 선을 가는 구멍에 꽂을 수 없다)
static bool wireFits(const SubSim& s, int from, int fromPort, int to, int toPort) {
    if (from < 0 || from >= (int)s.comps.size() || to < 0 || to >= (int)s.comps.size()) return false;
    const Comp& a = s.comps[from];
    const Comp& b = s.comps[to];
    if (fromPort < 0 || fromPort >= (int)a.outW.size()) return false;
    if (toPort   < 0 || toPort   >= (int)b.inW.size())  return false;
    return a.outW[fromPort] == b.inW[toPort];
}

static void addWire(SubSim& s, int from, int fromPort, int to, int toPort) {
    if (from == to) return;
    if (!wireFits(s, from, fromPort, to, toPort)) return;
    for (auto& w : s.wires)
        if (w.alive && w.to == to && w.toPort == toPort) w.alive = false;
    s.wires.push_back({ from, fromPort, to, toPort, true });
}
static void deleteComp(SubSim& s, int i) {
    s.comps[i].alive = false;
    for (auto& w : s.wires) if (w.from == i || w.to == i) w.alive = false;
}

// world 용 짧은 이름
static int  addComp(int type, int x = 0, int y = 0) { return addComp(world, type, x, y); }
static void addWire(int from, int fp, int to, int tp) { addWire(world, from, fp, to, tp); }

// ─────────────────────────────────────────────────────────────
// 시뮬레이션
//
// 모든 부품이 이번 틱 입력을 먼저 다 읽고, 그 다음 한꺼번에 출력을 바꾼다.
// 이 한 틱 지연 덕분에 되먹임 고리가 한 틱에 무한히 돌지 않고,
// NOR 두 개를 서로 물리면 값을 기억하는 래치가 된다.
//
// 커스텀 게이트는 바깥 입력을 안쪽 스위치에 넣고, 안쪽을 한 틱 돌리고,
// 안쪽 전구 값을 바깥 출력으로 뺀다. 그래서 상자 안에 상자도 된다.
// ─────────────────────────────────────────────────────────────

static void tickSub(SubSim& s) {
    int n = (int)s.comps.size();
    for (auto& c : s.comps) std::fill(c.in.begin(), c.in.end(), 0);

    for (const auto& w : s.wires) {
        if (!w.alive) continue;
        if (w.from < 0 || w.from >= n || w.to < 0 || w.to >= n) continue;
        Comp& src = s.comps[w.from]; Comp& dst = s.comps[w.to];
        if (!src.alive || !dst.alive) continue;
        if (w.fromPort < (int)src.out.size() && w.toPort < (int)dst.in.size()) {
            int wd = (w.toPort < (int)dst.inW.size()) ? dst.inW[w.toPort] : 1;
            dst.in[w.toPort] = maskTo(dst.in[w.toPort] | src.out[w.fromPort], wd);
        }
    }

    for (auto& c : s.comps) {
        if (!c.alive) continue;
        if (c.chipId >= 0) {
            SubSim& sub = *c.sub;
            // 바깥 입력을 안쪽 입력핀에 꽂고
            for (size_t k = 0; k < sub.inPorts.size() && k < c.in.size(); ++k) {
                int pin = sub.inPorts[k];
                if (pin >= 0 && pin < (int)sub.comps.size() && !sub.comps[pin].out.empty())
                    sub.comps[pin].out[0] = c.in[k];
            }
            tickSub(sub);
            // 안쪽 출력핀에 들어온 값을 바깥 출력으로 뺀다
            for (size_t k = 0; k < sub.outPorts.size() && k < c.nextOut.size(); ++k) {
                int pin = sub.outPorts[k];
                c.nextOut[k] = (pin >= 0 && pin < (int)sub.comps.size() && !sub.comps[pin].in.empty())
                               ? sub.comps[pin].in[0] : 0;
            }
        } else {
            Val a = c.in.size() > 0 ? c.in[0] : 0;
            Val b = c.in.size() > 1 ? c.in[1] : 0;
            switch (c.type) {
                // 스위치와 입력핀은 밖에서 정해 준 값을 그대로 들고 있는다
                case SWITCH:
                case SWBANK:
                case PIN_IN: if (!c.nextOut.empty()) c.nextOut[0] = c.out.empty() ? 0 : c.out[0]; break;
                case LAMP:
                case SCREEN:
                case PIN_OUT: break;                                   // 켜짐 = in[0]
                case T_NOT:  c.nextOut[0] = !a;         break;
                case T_AND:  c.nextOut[0] = a && b;     break;
                case T_OR:   c.nextOut[0] = a || b;     break;
                case T_XOR:  c.nextOut[0] = (a != 0) != (b != 0); break;
                case T_NAND: c.nextOut[0] = !(a && b);  break;
                case T_NOR:  c.nextOut[0] = !(a || b);  break;

                // 클럭은 제 박자대로 혼자 뒤집힌다. 값은 out 에 들고 있고
                // aux 의 아랫자리는 주기, 윗자리는 지금까지 센 틱이다.
                case CLOCK: {
                    int per = CLOCK_TICKS[std::clamp(c.aux, 0, CLOCK_N - 1)];
                    ++c.tickAcc;
                    Val now = c.out.empty() ? 0 : c.out[0];
                    if (c.tickAcc >= per) { c.tickAcc = 0; now = now ? 0 : 1; }
                    c.nextOut[0] = now;
                    break;
                }

                // 가는 선 여럿을 굵은 선 하나로. 0번 입력이 제일 아랫자리.
                case BUNDLE: {
                    Val v = 0;
                    for (int i = 0; i < (int)c.in.size() && i < WIDTH_MAX; ++i)
                        if (c.in[i]) v |= (Val)(1u << i);
                    c.nextOut[0] = maskTo(v, widthOf(c));
                    break;
                }
                // 굵은 선 하나를 가는 선 여럿으로
                case SPLIT: {
                    for (int i = 0; i < (int)c.nextOut.size() && i < WIDTH_MAX; ++i)
                        c.nextOut[i] = (a >> i) & 1;
                    break;
                }

                // 램: 주소·데이터·쓰기·클럭을 받는다.
                // 읽기는 늘 (주소 자리의 값이 그대로 나온다), 쓰기는 클럭이 올라갈 때만.
                case RAM: {
                    Val addr = a, dataIn = b;
                    Val we   = c.in.size() > 2 ? c.in[2] : 0;
                    Val clk  = c.in.size() > 3 ? c.in[3] : 0;
                    if (c.mem.size() != (size_t)RAM_WORDS) c.mem.assign(RAM_WORDS, 0);
                    if (we && clk && !c.lastClk)                  // 올라가는 순간에만 쓴다
                        c.mem[addr % RAM_WORDS] = maskTo(dataIn, RAM_DATAW);
                    c.lastClk = clk ? 1 : 0;
                    c.nextOut[0] = c.mem[addr % RAM_WORDS];
                    break;
                }
            }
            for (size_t i = 0; i < c.nextOut.size(); ++i)
                c.nextOut[i] = maskTo(c.nextOut[i], i < c.outW.size() ? c.outW[i] : 1);
        }
    }

    for (auto& c : s.comps) if (c.alive) c.out = c.nextOut;
}

// 부품의 켜짐 여부 (표시·읽기용). 출력 있으면 out[0], 전구는 in[0].
static bool lit(const Comp& c) {
    if (c.type == LAMP || c.type == PIN_OUT) return !c.in.empty() && c.in[0] != 0;
    return !c.out.empty() && c.out[0] != 0;
}
// 부품이 보여 줄 값 (굵은 선이면 수, 아니면 0/1)
static Val shownVal(const Comp& c) {
    if (c.type == LAMP || c.type == PIN_OUT || c.type == SPLIT)
        return c.in.empty() ? 0 : c.in[0];
    return c.out.empty() ? 0 : c.out[0];
}

// ─────────────────────────────────────────────────────────────
// 커스텀 게이트 만들기
// ─────────────────────────────────────────────────────────────

// 고른 부품들을 상자 하나로 묶는다. 안의 입력핀=입력 포트, 출력핀=출력 포트.
// 설계도에서 핀을 찾아 포트 차례를 정한다 (위→아래).
// 칩 안에 남은 스위치는 포트가 아니라 그냥 상수(늘 켜짐/꺼짐)다.
static void findPorts(SubSim& t) {
    t.inPorts.clear(); t.outPorts.clear();
    for (int i = 0; i < (int)t.comps.size(); ++i) {
        if (!t.comps[i].alive) continue;
        if (t.comps[i].type == PIN_IN)       t.inPorts.push_back(i);
        else if (t.comps[i].type == PIN_OUT) t.outPorts.push_back(i);
    }
    auto byPos = [&](int a, int b) {
        if (t.comps[a].y != t.comps[b].y) return t.comps[a].y < t.comps[b].y;
        return t.comps[a].x < t.comps[b].x;
    };
    std::sort(t.inPorts.begin(),  t.inPorts.end(),  byPos);
    std::sort(t.outPorts.begin(), t.outPorts.end(), byPos);
}

static void createChip(const std::vector<int>& sel, const std::string& name) {
    Chip ch; ch.name = name; ch.color = CHIP_COLORS[chips.size() % 8];
    SubSim& t = ch.tmpl;

    std::unordered_map<int, int> map;               // world 자리 → 템플릿 자리
    for (int wi : sel) {
        if (wi < 0 || wi >= (int)world.comps.size() || !world.comps[wi].alive) continue;
        map[wi] = (int)t.comps.size();
        Comp c = world.comps[wi];
        if (c.chipId >= 0 && c.sub) c.sub = std::make_shared<SubSim>(deepCopy(*c.sub));
        t.comps.push_back(std::move(c));
    }
    if (t.comps.empty()) return;

    for (const auto& w : world.wires) {             // 양 끝이 다 안에 든 선만
        if (!w.alive) continue;
        auto f = map.find(w.from), g = map.find(w.to);
        if (f != map.end() && g != map.end())
            t.wires.push_back({ f->second, w.fromPort, g->second, w.toPort, true });
    }

    findPorts(t);

    int minx = 1 << 30, miny = 1 << 30;
    for (int wi : sel)
        if (wi >= 0 && wi < (int)world.comps.size()) {
            minx = std::min(minx, world.comps[wi].x);
            miny = std::min(miny, world.comps[wi].y);
        }

    int chipId = (int)chips.size();
    chips.push_back(std::move(ch));
    for (int wi : sel) if (wi >= 0 && wi < (int)world.comps.size()) deleteComp(world, wi);
    addChip(world, chipId, minx, miny);
}

// ─────────────────────────────────────────────────────────────
// 커스텀 게이트 고치기
//
// 설계도를 고치면 이미 놓인 상자들도 다 새 설계도대로 바뀐다.
// 상자마다 속 회로를 새로 찍어 내고, 포트 수가 줄었으면 없어진 포트에
// 붙어 있던 선만 끊는다. 나머지 선은 그대로 둔다.
// ─────────────────────────────────────────────────────────────

// 이 묶음 안에서 chipId 를 쓰는 상자들을 새 설계도로 다시 찍는다.
// 끊어진 선 수를 돌려준다.
static int refreshChip(SubSim& s, int chipId) {
    int cut = 0;
    int ni = (int)chips[chipId].tmpl.inPorts.size();
    int no = (int)chips[chipId].tmpl.outPorts.size();

    for (int i = 0; i < (int)s.comps.size(); ++i) {
        Comp& c = s.comps[i];
        if (!c.alive) continue;
        if (c.chipId == chipId) {
            c.sub = std::make_shared<SubSim>(deepCopy(chips[chipId].tmpl));
            resizePorts(c);
            // 없어진 포트에 걸린 선을 끊는다
            for (auto& w : s.wires) {
                if (!w.alive) continue;
                if (w.to == i && w.toPort >= ni)       { w.alive = false; ++cut; }
                if (w.from == i && w.fromPort >= no)   { w.alive = false; ++cut; }
            }
        } else if (c.chipId >= 0 && c.sub) {
            cut += refreshChip(*c.sub, chipId);        // 상자 안의 상자도
        }
    }
    return cut;
}

// 고치는 중에는 그 칩의 설계도가 판 위에 올라와 있고, 원래 판은 여기 쌓아 둔다.
// 칩 안의 칩을 또 열 수 있어서 더미로 만들었다.
struct EditFrame { int chipId; SubSim board; };
static std::vector<EditFrame> editStack;

static bool editingChip(int id) {
    for (const auto& f : editStack) if (f.chipId == id) return true;
    return false;
}

// 판과 모든 설계도에 새 모습을 퍼뜨린다 (쌓아 둔 판까지)
static int applyChipEdit(int chipId) {
    int cut = refreshChip(world, chipId);
    for (int i = 0; i < (int)chips.size(); ++i)
        if (i != chipId && chips[i].alive) cut += refreshChip(chips[i].tmpl, chipId);
    for (auto& f : editStack) cut += refreshChip(f.board, chipId);
    return cut;
}

// 칩 고치기 시작 — 설계도를 판 위로 올린다
static bool beginEdit(int chipId) {
    if (chipId < 0 || chipId >= (int)chips.size() || !chips[chipId].alive) return false;
    if (editingChip(chipId)) return false;          // 자기 안에 자기를 넣을 수는 없다
    editStack.push_back({ chipId, std::move(world) });
    world = deepCopy(chips[chipId].tmpl);
    return true;
}

// 한 겹 나오기 — 판을 설계도로 되돌려 넣고 퍼뜨린다. 끊긴 선 수를 돌려준다.
static int endEdit() {
    if (editStack.empty()) return 0;
    EditFrame f = std::move(editStack.back());
    editStack.pop_back();
    chips[f.chipId].tmpl = deepCopy(world);
    findPorts(chips[f.chipId].tmpl);
    world = std::move(f.board);
    return applyChipEdit(f.chipId);
}

// ─────────────────────────────────────────────────────────────
// 커스텀 게이트 지우기
//
// 쓰는 중인 것은 못 지운다. 판에 놓인 상자와, 다른 칩의 설계도 속에 든 것을
// 둘 다 센다. 설계도 속은 재귀로 훑는다 — 칩 안에 칩이 있을 수 있다.
// ─────────────────────────────────────────────────────────────

static int countIn(const SubSim& s, int chipId) {
    int n = 0;
    for (const auto& c : s.comps) {
        if (!c.alive) continue;
        if (c.chipId == chipId) ++n;
        else if (c.chipId >= 0 && c.sub) n += countIn(*c.sub, chipId);
    }
    return n;
}

// 이 칩을 쓰고 있는 곳이 몇 군데인가 (판 + 살아 있는 다른 칩들의 설계도)
static int chipUses(int chipId) {
    int n = countIn(world, chipId);
    for (int i = 0; i < (int)chips.size(); ++i)
        if (i != chipId && chips[i].alive) n += countIn(chips[i].tmpl, chipId);
    for (const auto& f : editStack) n += countIn(f.board, chipId);
    return n;
}

// 지웠으면 true. 쓰는 중이면 아무것도 안 하고 false.
static bool deleteChip(int chipId) {
    if (chipId < 0 || chipId >= (int)chips.size() || !chips[chipId].alive) return false;
    if (editingChip(chipId)) return false;          // 고치는 중인 것은 못 지운다
    if (chipUses(chipId) > 0) return false;
    chips[chipId].alive = false;
    return true;
}

// ─────────────────────────────────────────────────────────────
// 고른 것 복사 / 돌리기
// ─────────────────────────────────────────────────────────────

// 오려 둔 조각. Ctrl+C 로 담고 Ctrl+V 로 꺼낸다.
struct Clip { std::vector<Comp> comps; std::vector<Wire> wires; };
static Clip clipboard;

// 고른 것들을 클립보드에 담는다 (판은 안 건드린다)
static int copyToClip(const std::vector<int>& sel) {
    clipboard.comps.clear(); clipboard.wires.clear();
    std::unordered_map<int, int> map;
    for (int i : sel) {
        if (i < 0 || i >= (int)world.comps.size() || !world.comps[i].alive) continue;
        Comp c = world.comps[i];
        if (c.chipId >= 0 && c.sub) c.sub = std::make_shared<SubSim>(deepCopy(*c.sub));
        map[i] = (int)clipboard.comps.size();
        clipboard.comps.push_back(std::move(c));
    }
    for (const auto& w : world.wires) {          // 양 끝이 다 담긴 선만
        if (!w.alive) continue;
        auto f = map.find(w.from), g = map.find(w.to);
        if (f != map.end() && g != map.end())
            clipboard.wires.push_back({ f->second, w.fromPort, g->second, w.toPort, true });
    }
    return (int)clipboard.comps.size();
}

// 담아 둔 것을 그 자리에 꺼낸다. 새로 생긴 자리를 돌려준다.
static std::vector<int> pasteClip(int atX, int atY) {
    std::vector<int> made;
    if (clipboard.comps.empty()) return made;
    int minx = 1 << 30, miny = 1 << 30;
    for (const auto& c : clipboard.comps) { minx = std::min(minx, c.x); miny = std::min(miny, c.y); }
    int base = (int)world.comps.size();
    for (const auto& c : clipboard.comps) {
        Comp n = c;
        if (c.chipId >= 0 && c.sub) n.sub = std::make_shared<SubSim>(deepCopy(*c.sub));
        n.x = atX + (c.x - minx); n.y = atY + (c.y - miny);
        made.push_back((int)world.comps.size());
        world.comps.push_back(std::move(n));
    }
    for (const auto& w : clipboard.wires)
        world.wires.push_back({ base + w.from, w.fromPort, base + w.to, w.toPort, true });
    return made;
}

// 고른 부품들을 그대로 복제한다. 둘 다 안에 든 선도 같이 복사된다.
// 새로 생긴 것들의 자리를 돌려주니 그대로 다시 고른 상태가 된다.
static std::vector<int> duplicate(const std::vector<int>& sel, int offx, int offy) {
    std::unordered_map<int, int> map;
    std::vector<int> made;
    for (int i : sel) {
        if (i < 0 || i >= (int)world.comps.size() || !world.comps[i].alive) continue;
        Comp c = world.comps[i];
        if (c.chipId >= 0 && c.sub) c.sub = std::make_shared<SubSim>(deepCopy(*c.sub));
        c.x += offx; c.y += offy;
        map[i] = (int)world.comps.size();
        made.push_back((int)world.comps.size());
        world.comps.push_back(std::move(c));
    }
    // 양 끝이 다 복사 대상 안에 있는 선만 같이 복제한다
    int n = (int)world.wires.size();
    for (int k = 0; k < n; ++k) {
        const Wire& w = world.wires[k];
        if (!w.alive) continue;
        auto f = map.find(w.from), g = map.find(w.to);
        if (f != map.end() && g != map.end())
            world.wires.push_back({ f->second, w.fromPort, g->second, w.toPort, true });
    }
    return made;
}

// 크기는 글꼴을 알아야 나와서 아래쪽에 있다
static int compW(const Comp& c);
static int compH(const Comp& c);

// 고른 부품들의 방향을 한 칸씩 돌린다.
static void rotateSel(const std::vector<int>& sel, int step) {
    for (int i : sel)
        if (i >= 0 && i < (int)world.comps.size() && world.comps[i].alive) {
            Comp& c = world.comps[i];
            // 돌아도 가운데가 그대로이게 자리를 맞춘다 (가로·세로가 바뀌므로)
            int cx = c.x + compW(c) / 2, cy = c.y + compH(c) / 2;
            c.rot = (c.rot + step) & 3;
            c.x = cx - compW(c) / 2; c.y = cy - compH(c) / 2;
        }
}

// ─────────────────────────────────────────────────────────────
// 색 (모래에서 가져옴)
// ─────────────────────────────────────────────────────────────

static inline uint32_t shade(uint32_t col, int off) {
    int r = std::clamp((int)((col >> 16) & 0xFF) + off, 0, 255);
    int g = std::clamp((int)((col >> 8)  & 0xFF) + off, 0, 255);
    int b = std::clamp((int)( col        & 0xFF) + off, 0, 255);
    return (uint32_t(r) << 16) | (uint32_t(g) << 8) | uint32_t(b);
}
static uint32_t blend(uint32_t a, uint32_t b, int f) {   // f: 0~256
    int r = ((a >> 16) & 0xFF) + (((int)((b >> 16) & 0xFF) - (int)((a >> 16) & 0xFF)) * f) / 256;
    int g = ((a >> 8)  & 0xFF) + (((int)((b >> 8)  & 0xFF) - (int)((a >> 8)  & 0xFF)) * f) / 256;
    int bl = (a & 0xFF)        + (((int)(b & 0xFF)        - (int)(a & 0xFF))        * f) / 256;
    return (uint32_t(r) << 16) | (uint32_t(g) << 8) | uint32_t(bl);
}

static const uint32_t COL_BG    = 0x14161C;
static const uint32_t COL_PANEL = 0x1B1E26;
static const uint32_t COL_LINE  = 0x2C303C;
static const uint32_t COL_ON    = 0x46D67A;
static const uint32_t COL_OFF   = 0x3A3E48;
static const uint32_t COL_TEXT  = 0xE6E6EE;
static const uint32_t COL_DIM   = 0x8A8E9A;
static const uint32_t COL_SEL   = 0xE0C060;

// ─────────────────────────────────────────────────────────────
// 글자 그리기 (모래에서 가져옴 — stb_truetype)
// ─────────────────────────────────────────────────────────────

// 환경변수는 새 이름(SEMTLE_*) 을 먼저 보고, 없으면 옛 이름(LOGIC_*) 도 받는다.
static const char* env2(const char* now, const char* old) {
    const char* v = SDL_getenv(now);
    return v ? v : SDL_getenv(old);
}

static const char* FONT_PATHS[] = {
    "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/noto-cjk/NotoSansCJK-Medium.ttc",
    "/usr/share/fonts/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/TTF/NanumGothic.ttf",
};
static std::vector<unsigned char> fontData;
static stbtt_fontinfo             font;
static bool                       fontOK = false;

static bool loadFont() {
    const char* env = env2("SEMTLE_FONT", "LOGIC_FONT");
    const char* tries[8]; int n = 0;
    if (env) tries[n++] = env;
    for (const char* p : FONT_PATHS) tries[n++] = p;
    for (int i = 0; i < n; ++i) {
        SDL_RWops* f = SDL_RWFromFile(tries[i], "rb");
        if (!f) continue;
        Sint64 sz = SDL_RWsize(f);
        fontData.resize((size_t)sz);
        SDL_RWread(f, fontData.data(), 1, (size_t)sz);
        SDL_RWclose(f);
        int off = stbtt_GetFontOffsetForIndex(fontData.data(), 0);
        if (off >= 0 && stbtt_InitFont(&font, fontData.data(), off)) { fontOK = true; return true; }
    }
    std::fprintf(stderr, "폰트를 못 찾음. SEMTLE_FONT 환경변수로 지정하면 된다.\n");
    return false;
}

struct Glyph { SDL_Texture* tex; int w, h, bx, by, adv; };
static std::unordered_map<uint64_t, Glyph> glyphCache;

static const Glyph* getGlyph(SDL_Renderer* ren, int px, uint32_t cp) {
    uint64_t key = (uint64_t(px) << 32) | cp;
    auto it = glyphCache.find(key);
    if (it != glyphCache.end()) return &it->second;
    Glyph g{};
    float scale = stbtt_ScaleForPixelHeight(&font, (float)px);
    int adv, lsb; stbtt_GetCodepointHMetrics(&font, (int)cp, &adv, &lsb);
    g.adv = (int)(adv * scale + 0.5f);
    int x0, y0, x1, y1; stbtt_GetCodepointBitmapBox(&font, (int)cp, scale, scale, &x0, &y0, &x1, &y1);
    g.w = x1 - x0; g.h = y1 - y0; g.bx = x0; g.by = y0;
    if (g.w > 0 && g.h > 0) {
        std::vector<unsigned char> cov((size_t)g.w * g.h);
        stbtt_MakeCodepointBitmap(&font, cov.data(), g.w, g.h, g.w, scale, scale, (int)cp);
        std::vector<uint32_t> px32((size_t)g.w * g.h);
        for (size_t i = 0; i < px32.size(); ++i)
            px32[i] = (uint32_t(cov[i]) << 24) | 0x00FFFFFFu;
        g.tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC, g.w, g.h);
        SDL_UpdateTexture(g.tex, nullptr, px32.data(), g.w * 4);
        SDL_SetTextureBlendMode(g.tex, SDL_BLENDMODE_BLEND);
    }
    return &(glyphCache[key] = g);
}
static uint32_t nextCp(const char*& s) {
    unsigned char c = (unsigned char)*s++;
    if (c < 0x80) return c;
    int extra = (c >= 0xF0) ? 3 : (c >= 0xE0) ? 2 : 1;
    uint32_t cp = c & (0x3F >> extra);
    while (extra-- && (*s & 0xC0) == 0x80) cp = (cp << 6) | (*s++ & 0x3F);
    return cp;
}
// px 는 1배 기준 크기다. 실제로 그릴 때 화면 배율만큼 늘린다.
static int textWidth(int px, const char* s) {
    if (!fontOK) return 0;
    px = S(px);
    float scale = stbtt_ScaleForPixelHeight(&font, (float)px);
    int w = 0;
    while (*s) { int adv, lsb; stbtt_GetCodepointHMetrics(&font, (int)nextCp(s), &adv, &lsb);
                 w += (int)(adv * scale + 0.5f); }
    return w;
}
static void drawText(SDL_Renderer* ren, int x, int y, int px, uint32_t col, const char* s) {
    if (!fontOK) return;
    px = S(px);
    float scale = stbtt_ScaleForPixelHeight(&font, (float)px);
    int asc, desc, gap; stbtt_GetFontVMetrics(&font, &asc, &desc, &gap);
    int baseline = y + (int)(asc * scale + 0.5f);
    while (*s) {
        const Glyph* g = getGlyph(ren, px, nextCp(s));
        if (g->tex) {
            SDL_SetTextureColorMod(g->tex, (col >> 16) & 0xFF, (col >> 8) & 0xFF, col & 0xFF);
            SDL_Rect d{ x + g->bx, baseline + g->by, g->w, g->h };
            SDL_RenderCopy(ren, g->tex, nullptr, &d);
        }
        x += g->adv;
    }
}
static void drawTextC(SDL_Renderer* ren, int cx, int y, int px, uint32_t col, const char* s) {
    drawText(ren, cx - textWidth(px, s) / 2, y, px, col, s);
}

// ─────────────────────────────────────────────────────────────
// 화면 보기 (확대·이동)
//
// 부품 좌표는 판 좌표(월드)로 들고 있고, 그릴 때만 화면 좌표로 바꾼다.
// 마우스는 들어오자마자 판 좌표로 바꿔서, 히트 테스트는 배율을 몰라도 된다.
// ─────────────────────────────────────────────────────────────

static float viewZoom = 1.0f;
static float viewX = 0, viewY = 0;      // 캔버스 왼위에 오는 판 좌표

static const float ZOOM_MIN = 0.25f, ZOOM_MAX = 3.0f;

// 판이 시작하는 화면 x. Tab 으로 도구판을 숨기면 0 이 된다. (아래에 정의)
static int canvasL();

static int  w2sX(float wx) { return canvasL() + (int)((wx - viewX) * viewZoom); }
static int  w2sY(float wy) { return             (int)((wy - viewY) * viewZoom); }
static float s2wX(int sx)  { return viewX + (sx - canvasL()) / viewZoom; }
static float s2wY(int sy)  { return viewY + sy / viewZoom; }
static int  w2sLen(int len) { return std::max(1, (int)(len * viewZoom)); }

// 화면의 이 점을 판의 이 점에 고정한 채 배율만 바꾼다 (커서 밑이 안 밀린다)
static void zoomAt(float nz, int sx, int sy) {
    nz = std::clamp(nz, ZOOM_MIN, ZOOM_MAX);
    float wx = s2wX(sx), wy = s2wY(sy);
    viewZoom = nz;
    viewX = wx - (sx - canvasL()) / viewZoom;
    viewY = wy - sy / viewZoom;
}

// ─────────────────────────────────────────────────────────────
// 부품 크기·포트 자리
//
// rot: 0 = 출력이 오른쪽, 1 = 아래, 2 = 왼쪽, 3 = 위.
// 홀수로 돌면 상자의 가로·세로가 바뀐다.
// ─────────────────────────────────────────────────────────────

// 안 돌린 상태의 크기
static int baseW(const Comp& c) {
    if (c.type == SCREEN) return std::max(GW_MIN, widthOf(c) * 14 + 20);
    if (isBitBank(c)) return std::max(GW_MIN, textWidth(13, compName(c)) + 24);
    return std::max(GW_MIN, textWidth(15, compName(c)) + 26);
}
static int baseH(const Comp& c) {
    if (c.type == SCREEN) return std::max(48, screenRows(c) * 14 + 28);
    if (isBitBank(c)) return std::max(48, widthOf(c) * 20 + 34);
    if (isFolded(c)) return 52;          // 접으면 게이트만 한 크기
    return std::max(48, std::max(nIn(c), nOut(c)) * 20 + 12);
}
// 판 위에서 차지하는 크기 (돌린 걸 반영)
static int compW(const Comp& c) { return (c.rot & 1) ? baseH(c) : baseW(c); }
static int compH(const Comp& c) { return (c.rot & 1) ? baseW(c) : baseH(c); }

// 어느 변에 포트를 늘어놓을지: side 0=오른, 1=아래, 2=왼, 3=위
static void portOnSide(const Comp& c, int side, int i, int n, int& px, int& py) {
    int w = compW(c), h = compH(c);
    n = std::max(1, n);
    switch (side) {
        case 0: px = c.x + w;             py = c.y + h * (i + 1) / (n + 1); break;
        case 1: px = c.x + w * (i+1)/(n+1); py = c.y + h;                   break;
        case 2: px = c.x;                 py = c.y + h * (i + 1) / (n + 1); break;
        default:px = c.x + w * (i+1)/(n+1); py = c.y;                       break;
    }
}
static void outPort(const Comp& c, int port, int& px, int& py) {
    portOnSide(c, c.rot & 3, port, nOut(c), px, py);
}
static void inPort(const Comp& c, int port, int& px, int& py) {
    portOnSide(c, (c.rot + 2) & 3, port, nIn(c), px, py);   // 출력의 맞은편
}
// 선이 이 포트에서 뻗어 나가는 방향 (곡선 접선용)
static void sideDir(int side, int& dx, int& dy) {
    switch (side) {
        case 0: dx =  1; dy =  0; break;
        case 1: dx =  0; dy =  1; break;
        case 2: dx = -1; dy =  0; break;
        default:dx =  0; dy = -1; break;
    }
}
static void outDir(const Comp& c, int& dx, int& dy) { sideDir(c.rot & 3, dx, dy); }
static void inDir(const Comp& c, int& dx, int& dy)  { sideDir((c.rot + 2) & 3, dx, dy); }

// ─────────────────────────────────────────────────────────────
// 그리기 헬퍼
// ─────────────────────────────────────────────────────────────

struct Rect {
    int x, y, w, h;
    bool has(int px, int py) const { return px >= x && px < x + w && py >= y && py < y + h; }
};
static void setCol(SDL_Renderer* r, uint32_t c, uint8_t a = 255) {
    SDL_SetRenderDrawColor(r, (c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF, a);
}
static void fillRect(SDL_Renderer* r, Rect b, uint32_t c, uint8_t a = 255) {
    setCol(r, c, a); SDL_Rect s{ b.x, b.y, b.w, b.h }; SDL_RenderFillRect(r, &s);
}
static void frameRect(SDL_Renderer* r, Rect b, uint32_t c, uint8_t a = 255) {
    setCol(r, c, a); SDL_Rect s{ b.x, b.y, b.w, b.h }; SDL_RenderDrawRect(r, &s);
}
static void fillCircle(SDL_Renderer* r, int cx, int cy, int rad, uint32_t c, uint8_t a = 255) {
    setCol(r, c, a);
    for (int dy = -rad; dy <= rad; ++dy) {
        int dx = (int)std::sqrt((double)rad * rad - dy * dy);
        SDL_RenderDrawLine(r, cx - dx, cy + dy, cx + dx, cy + dy);
    }
}
static void thickLine(SDL_Renderer* r, int x0, int y0, int x1, int y1, int th, uint32_t c) {
    setCol(r, c);
    for (int o = -th / 2; o <= th / 2; ++o) {
        SDL_RenderDrawLine(r, x0, y0 + o, x1, y1 + o);
        SDL_RenderDrawLine(r, x0 + o, y0, x1 + o, y1);
    }
}
// 양 끝에서 포트가 보는 쪽으로 뻗어 나갔다가 이어진다.
// (d0x,d0y) 는 나가는 방향, (d1x,d1y) 는 들어오는 쪽이 보는 방향.
static void bezierD(int x0, int y0, int d0x, int d0y,
                    int x1, int y1, int d1x, int d1y, int out[][2], int n) {
    int dist = std::abs(x1 - x0) + std::abs(y1 - y0);
    int tang = std::clamp(dist / 2, 40, 200);
    int cx0 = x0 + d0x * tang, cy0 = y0 + d0y * tang;
    int cx1 = x1 + d1x * tang, cy1 = y1 + d1y * tang;
    for (int i = 0; i < n; ++i) {
        float t = (float)i / (n - 1), u = 1 - t;
        float b0 = u*u*u, b1 = 3*u*u*t, b2 = 3*u*t*t, b3 = t*t*t;
        out[i][0] = (int)(b0*x0 + b1*cx0 + b2*cx1 + b3*x1);
        out[i][1] = (int)(b0*y0 + b1*cy0 + b2*cy1 + b3*y1);
    }
}
// 선 하나의 곡선을 판 좌표로 뽑는다
static void wireCurve(const Wire& w, int out[][2], int n);

// ─────────────────────────────────────────────────────────────
// 판 (왼쪽) — 도구 목록은 붙박이 + 커스텀 게이트
// ─────────────────────────────────────────────────────────────

// 도구 자리: 0 = 손, 1..TYPE_N = 붙박이, 그 뒤 = 살아 있는 칩들
static int toolCount() { return 1 + TYPE_N + (int)liveChips().size(); }

// 목록 찾기. 여기 적은 글이 이름에 든 부품만 보인다.
static std::string toolFind;
static bool findOn = false;         // 찾는 글을 치는 중인가

static const char* toolNameOf(int i);   // 앞선언

// 지금 보여 줄 도구들 (canonical 번호). 찾는 글이 있으면 걸러진다.
static std::vector<int> shownTools() {
    std::vector<int> v;
    int n = toolCount();
    for (int i = 0; i < n; ++i) {
        if (i == 0) { v.push_back(i); continue; }        // 손은 늘 보인다
        if (toolFind.empty()) { v.push_back(i); continue; }
        std::string nm = toolNameOf(i);
        if (nm.find(toolFind) != std::string::npos) v.push_back(i);
    }
    return v;
}
static int shownCount() { return (int)shownTools().size(); }
// 부품이 많아지면 줄을 좁힌다. 그래도 넘치면 휠로 굴려 본다.
static int toolTop()  { return S(72); }   // 찾기 칸 아래부터
static int toolBot()  { return (WIN_H - S(84)) - S(6); }   // 아래 단추들 위
// 줄 높이는 고정이다. 예전엔 다 들어가게 줄였는데, 그러면 칩이 쌓일수록
// 줄이 빽빽해지고 굴릴 일이 없어져 휠이 안 먹는 것처럼 보였다.
// 이제 넘치면 굴려서 본다.
static int toolPitch() { return S(28); }
static int toolScroll = 0;        // 목록을 얼마나 굴렸나 (픽셀)

// 다 못 보여 줄 때 굴릴 수 있는 최대
static int toolScrollMax() {
    int need = shownCount() * toolPitch();
    return std::max(0, need - (toolBot() - toolTop()));
}
static void clampToolScroll() { toolScroll = std::clamp(toolScroll, 0, toolScrollMax()); }

// row = 화면에 보이는 순서 (걸러진 뒤)
static Rect toolRect(int row) {
    int p = toolPitch();
    return { S(10), toolTop() + row * p - toolScroll, PANEL_W - S(20) - S(8), p - S(3) };
}
static bool toolVisible(int row) {
    Rect r = toolRect(row);
    return r.y >= toolTop() - r.h / 2 && r.y + r.h <= toolBot() + r.h / 2;
}

// 찾기 칸과 굴림 막대
static Rect findBox() { return { S(10), S(38), PANEL_W - S(20), S(24) }; }
static Rect scrollTrack() {
    return { PANEL_W - S(14), toolTop(), S(6), toolBot() - toolTop() };
}
static Rect scrollThumb() {
    Rect t = scrollTrack();
    int need = shownCount() * toolPitch(), room = toolBot() - toolTop();
    if (need <= room) return { t.x, t.y, t.w, t.h };
    int hh = std::max(S(24), t.h * room / std::max(1, need));
    int maxOff = toolScrollMax();
    int y = t.y + (maxOff ? (t.h - hh) * toolScroll / maxOff : 0);
    return { t.x, y, t.w, hh };
}

// 도구 자리가 칩이면 그 chipId, 아니면 -1
static int toolChip(int i) {
    if (i <= TYPE_N) return -1;
    std::vector<int> lc = liveChips();
    int k = i - 1 - TYPE_N;
    return (k >= 0 && k < (int)lc.size()) ? lc[k] : -1;
}
static const char* toolNameOf(int i);

static const char* toolName(int i) {
    if (i == 0) return "손 (고르기)";
    if (i <= TYPE_N) return TYPES[i - 1].name;
    int c = toolChip(i);
    return c >= 0 ? chips[c].name.c_str() : "?";
}
static const char* toolNameOf(int i) { return toolName(i); }

static bool toolHasColor(int i) { return i > 0; }
static uint32_t toolColor(int i) {
    if (i >= 1 && i <= TYPE_N) return TYPES[i - 1].color;
    int c = toolChip(i);
    return c >= 0 ? chips[c].color : 0x555555;
}
// 칩 줄 오른쪽 끝의 지우기 단추
static Rect toolDelBtn(int i) {
    Rect r = toolRect(i);
    int sz = std::min(S(17), r.h - S(4));
    return { r.x + r.w - sz - S(5), r.y + (r.h - sz) / 2, sz, sz };
}
// 그 왼쪽의 고치기 단추
static Rect toolEditBtn(int i) {
    Rect d = toolDelBtn(i);
    return { d.x - d.w - S(4), d.y, d.w, d.h };
}
// 도구 i 가 놓는 부품을 판에 만든다
static int placeTool(int i, int x, int y) {
    if (i >= 1 && i <= TYPE_N) return addComp(world, i - 1, x, y);
    int c = toolChip(i);
    return c >= 0 ? addChip(world, c, x, y) : -1;
}

static Rect btnBundle() { return { S(10), WIN_H - S(84), PANEL_W - S(20), S(30) }; }
static Rect btnClear()  { return { S(10), WIN_H - S(46), PANEL_W - S(20), S(30) }; }

// 선 하나의 곡선 (판 좌표). 양 끝 포트가 보는 방향으로 뻗는다.
static void wireCurve(const Wire& w, int out[][2], int n) {
    const Comp& a = world.comps[w.from];
    const Comp& b = world.comps[w.to];
    int x0, y0, x1, y1, d0x, d0y, d1x, d1y;
    outPort(a, w.fromPort, x0, y0); outDir(a, d0x, d0y);
    inPort(b, w.toPort, x1, y1);    inDir(b, d1x, d1y);
    bezierD(x0, y0, d0x, d0y, x1, y1, d1x, d1y, out, n);
}

// ─────────────────────────────────────────────────────────────
// 히트 테스트
// ─────────────────────────────────────────────────────────────

static bool outPortAt(int mx, int my, int& comp, int& port) {
    for (int i = (int)world.comps.size() - 1; i >= 0; --i) {
        if (!world.comps[i].alive) continue;
        int no = nOut(world.comps[i]);
        for (int p = 0; p < no; ++p) {
            int px, py; outPort(world.comps[i], p, px, py);
            if ((mx-px)*(mx-px) + (my-py)*(my-py) <= PORT_HIT*PORT_HIT) { comp = i; port = p; return true; }
        }
    }
    return false;
}
static bool inPortAt(int mx, int my, int& comp, int& port) {
    for (int i = (int)world.comps.size() - 1; i >= 0; --i) {
        if (!world.comps[i].alive) continue;
        int ni = nIn(world.comps[i]);
        for (int p = 0; p < ni; ++p) {
            int px, py; inPort(world.comps[i], p, px, py);
            if ((mx-px)*(mx-px) + (my-py)*(my-py) <= PORT_HIT*PORT_HIT) { comp = i; port = p; return true; }
        }
    }
    return false;
}
// 스위치묶음에서 누른 비트 자리 (없으면 -1)
static int swBankBitAt(const Comp& c, int wx2, int wy2) {
    if (!isBitBank(c)) return -1;
    int w = compW(c), h = compH(c);
    if (wx2 < c.x || wx2 >= c.x + w || wy2 < c.y || wy2 >= c.y + h) return -1;
    int wd = widthOf(c), pad = 4;
    int top = (c.type == PIN_IN) ? 18 : 0;
    int rowH = (h - 20 - pad - top) / std::max(1, wd);
    if (rowH <= 0) return -1;
    int fromBottom = (c.y + h - 20) - wy2;
    if (fromBottom < 0) return -1;
    int i = fromBottom / rowH;
    return (i >= 0 && i < wd) ? i : -1;
}

static int compAt(int mx, int my) {
    for (int i = (int)world.comps.size() - 1; i >= 0; --i) {
        if (!world.comps[i].alive) continue;
        Rect b{ world.comps[i].x, world.comps[i].y, compW(world.comps[i]), compH(world.comps[i]) };
        if (b.has(mx, my)) return i;
    }
    return -1;
}
static int wireAt(int mx, int my) {
    int pts[24][2];
    // 배율이 작을수록 판 좌표에서 넉넉히 잡아야 화면에선 같은 두께로 집힌다
    int tol = (int)(7 / std::max(0.25f, viewZoom));
    for (int i = (int)world.wires.size() - 1; i >= 0; --i) {
        const Wire& w = world.wires[i];
        if (!w.alive) continue;
        if (!world.comps[w.from].alive || !world.comps[w.to].alive) continue;
        wireCurve(w, pts, 24);
        for (int k = 0; k < 24; ++k) {
            int dx = mx - pts[k][0], dy = my - pts[k][1];
            if (dx*dx + dy*dy <= tol*tol) return i;
        }
    }
    return -1;
}

// ─────────────────────────────────────────────────────────────
// 학습 모드 화면 (오른쪽에 설명과 진리표)
// ─────────────────────────────────────────────────────────────

static Rect lessonPanel() { return { WIN_W - LES_W, 0, LES_W, WIN_H }; }
static Rect btnGrade()    { return { WIN_W - LES_W + S(14), WIN_H - S(58), LES_W - S(28), S(40) }; }
// 앞 · 힌트 · 뒤 세 칸
static Rect lesRow(int i) {
    int w = (LES_W - S(28) - S(12)) / 3;
    return { WIN_W - LES_W + S(14) + i * (w + S(6)), WIN_H - S(104), w, S(32) };
}
static Rect btnPrevLes()  { return lesRow(0); }
static Rect btnHint()     { return lesRow(1); }
static Rect btnNextLes()  { return lesRow(2); }

static bool hintOn = false;      // 지금 단계의 힌트를 펼쳤나
// ─────────────────────────────────────────────────────────────
// 학습 모드
//
// 단계마다 짧은 설명과 진리표를 준다. 판에 입력핀·출력핀을 놓고 회로를 짜면
// 채점기가 입력을 모든 경우로 돌려 보며 출력이 표와 맞는지 본다.
// 통과하면 그 회로가 칩이 되어 다음 단계에서 부품으로 쓰인다.
// ─────────────────────────────────────────────────────────────

// 지금 어느 화면인가
enum Screen { SC_MENU, SC_LEARN, SC_SANDBOX, SC_PICK };
static int screen = SC_SANDBOX;

static int lessonAt = 0;      // 지금 푸는 단계
static int lessonDone = 0;    // 여기까지 통과했다

static bool uiOn = true;      // 좌우 판을 보여 줄까 (Tab)

// 과정을 고치면(단계를 쪼개거나 순서를 바꾸면) 저장된 진도가 엉뚱한 단계를 가리킨다.
// 이 번호를 올려 두면 진도만 처음으로 되돌리고, 얻어 둔 칩은 그대로 둔다.
static const int COURSE_VER = 5;

// 기억하는 회로는 진리표로 못 잰다. 값을 차례로 넣어 보며 그때그때 확인해야 한다.
// 한 칸이 "이 값들을 넣고 몇 틱 돌린 뒤, 이 출력이 이 값이어야 한다" 를 뜻한다.
struct Step {
    Val  in[4];        // 입력핀에 넣을 값
    Val  want[3];      // 출력핀에서 바라는 값
    uint8_t check;     // 어느 출력을 볼지 (비트 0 = 첫째 출력 …). 0 이면 안 본다
    uint8_t ticks;     // 넣고 몇 틱 돌릴지 (0 이면 기본값)
    const char* note;  // 못 맞췄을 때 같이 보여 줄 말
};

static const int LES_IN_MAX = 4, LES_OUT_MAX = 3;

struct Lesson {
    const char* name;      // 통과하면 이 이름의 칩이 된다
    const char* title;
    const char* text;      // 설명 (\n 으로 줄 나눔)
    int  nIn, nOut;
    const char* inName[LES_IN_MAX];
    const char* outName[LES_OUT_MAX];
    // 진리표: 입력 조합 2^nIn 개마다 원하는 출력 비트들 (script 가 있으면 안 쓴다)
    uint8_t want[8][2];
    const char* allow;     // 이 단계에서 쓸 수 있는 부품 (빈 칸이면 다)
    bool  bank;            // 통과하면 칩으로 남길까 (이해 단계는 안 남긴다)
    const char* hint;      // 막혔을 때 보여 줄 귀띔 (답을 다 알려 주지는 않는다)
    // 여기부터는 뒤에 생긴 것들 — 안 적으면 0 이 되어 예전처럼 돈다
    int  inW[LES_IN_MAX];  // 입력핀 폭 (0 이면 1)
    int  outW[LES_OUT_MAX];
    const Step* script;    // 있으면 진리표 대신 이걸로 잰다
    int  nStep;
    const Val* program;    // 판에 놓인 램에 미리 넣어 둘 프로그램
    int  nProg;
};

// ── 기억하는 단계들의 검사 대본 ──
// 셋·리셋을 잠깐씩 주고, 뗀 뒤에도 값이 남아 있는지 본다.
static const Step LAT_SCRIPT[] = {
    { {0,0}, {0}, 0, 0, nullptr },                       // 처음엔 아무 값이나
    { {1,0}, {1}, 1, 0, "셋을 켜면 Q 가 켜져야 한다" },
    { {0,0}, {1}, 1, 0, "셋을 떼도 켜진 채로 있어야 한다" },
    { {0,1}, {0}, 1, 0, "리셋을 켜면 꺼져야 한다" },
    { {0,0}, {0}, 1, 0, "리셋을 떼도 꺼진 채로 있어야 한다" },
    { {1,0}, {1}, 1, 0, "다시 셋을 켜면 켜져야 한다" },
};

// 받아라가 켜져 있을 때만 값이 통하고, 꺼지면 붙잡고 있어야 한다.
static const Step DLAT_SCRIPT[] = {
    { {0,1}, {0}, 1, 0, "값 0 을 받는다" },
    { {1,1}, {1}, 1, 0, "받아라가 켜져 있으면 값이 그대로 통한다" },
    { {1,0}, {1}, 1, 0, "받아라를 끄면 그 값을 붙잡는다" },
    { {0,0}, {1}, 1, 0, "꺼져 있으면 값이 바뀌어도 안 바뀐다" },
    { {0,1}, {0}, 1, 0, "다시 켜면 새 값을 받는다" },
    { {1,0}, {0}, 1, 0, "끈 뒤에는 값이 와도 안 통한다" },
    { {1,1}, {1}, 1, 0, "켜면 다시 통한다" },
};

// 마스터-슬레이브: 클럭이 올라가는 그 순간의 값만 잡는다.
// D래치와 다른 점은 4번째 — 올라간 뒤에 값이 바뀌어도 안 통한다.
static const Step FF_SCRIPT[] = {
    { {0,1}, {0}, 0, 0, nullptr },      // 먼저 0 을 한 번 밀어 넣어 자리를 잡는다
    { {0,0}, {0}, 1, 0, "처음엔 0" },
    { {1,0}, {0}, 1, 0, "클럭이 아직 안 올라갔으니 그대로" },
    { {1,1}, {1}, 1, 0, "올라가는 순간의 값을 잡는다" },
    { {0,1}, {1}, 1, 0, "올라간 뒤에 값이 바뀌어도 안 통해야 한다" },
    { {0,0}, {1}, 1, 0, "내려가도 그대로" },
    { {0,1}, {0}, 1, 0, "다음에 올라갈 때 새 값" },
    { {1,1}, {0}, 1, 0, "또 안 통한다" },
};

// 8비트짜리. 값이 수로 오간다.
static const Step REG_SCRIPT[] = {
    { {0,1},   {0},   0, 0, nullptr },  // 0 을 한 번 밀어 넣어 자리를 잡는다
    { {0,0},   {0},   1, 0, "처음엔 0" },
    { {181,0}, {0},   1, 0, "클럭이 안 올라갔으니 아직" },
    { {181,1}, {181}, 1, 0, "올라가는 순간 181 을 잡는다" },
    { {7,1},   {181}, 1, 0, "올라간 뒤 값이 바뀌어도 그대로" },
    { {7,0},   {181}, 1, 0, "내려가도 그대로" },
    { {7,1},   {7},   1, 0, "다음에 올라갈 때 7" },
    { {255,1}, {7},   1, 0, "또 안 통한다" },
};

// 8비트 덧셈기: 두 수를 더해 결과와 자리올림을 낸다
static const Step ADD8_SCRIPT[] = {
    { {0,0},     {0,0},     3, 0, "0+0" },
    { {1,1},     {2,0},     3, 0, "1+1 = 2" },
    { {181,7},   {188,0},   3, 0, "181+7 = 188" },
    { {200,100}, {44,1},    3, 0, "200+100 = 300 → 44 에 자리올림 1" },
    { {255,1},   {0,1},     3, 0, "255+1 = 256 → 0 에 자리올림 1" },
    { {255,255}, {254,1},   3, 0, "255+255 = 510 → 254 에 자리올림 1" },
};

// 세는 상자: 클럭이 올라갈 때마다 1씩 는다. 되돌리기를 켜면 0 으로.
static const Step CNT_SCRIPT[] = {
    { {0,1}, {0}, 0, 0, nullptr },                    // 되돌리기를 켠 채로
    { {1,1}, {0}, 1, 0, "되돌리는 중에 클럭이 올라가면 0" },
    { {0,0}, {0}, 1, 0, "되돌리기를 뗀다" },
    { {1,0}, {1}, 1, 0, "클럭이 올라가면 1" },
    { {0,0}, {1}, 1, 0, "내려갈 땐 그대로" },
    { {1,0}, {2}, 1, 0, "또 올라가면 2" },
    { {0,0}, {2}, 1, 0, "그대로" },
    { {1,0}, {3}, 1, 0, "3" },
    { {0,0}, {3}, 1, 0, "그대로" },
    { {0,1}, {3}, 0, 0, nullptr },                    // 되돌리기 켜고
    { {1,1}, {0}, 1, 0, "다시 0 으로" },
};

// 계산기: 빼기를 켜면 A-B, 끄면 A+B
static const Step ALU_SCRIPT[] = {
    { {5,3,0},   {8},   1, 0, "5+3 = 8" },
    { {5,3,1},   {2},   1, 0, "5-3 = 2" },
    { {200,100,0},{44}, 1, 0, "200+100 은 넘쳐서 44" },
    { {10,10,1}, {0},   1, 0, "10-10 = 0" },
    { {3,5,1},   {254}, 1, 0, "3-5 는 음수라 254 (2의 보수)" },
    { {255,1,0}, {0},   1, 0, "255+1 = 0" },
};

// 명령 해독기: 명령 번호를 받아 어느 선을 켤지 정한다
static const Step DEC_SCRIPT[] = {
    { {0}, {1,0,0}, 7, 0, "0 = 멈춤" },
    { {1}, {0,1,0}, 7, 0, "1 = 불러오기" },
    { {2}, {0,0,1}, 7, 0, "2 = 더하기" },
    { {3}, {0,0,0}, 7, 0, "3 = 아무것도 아님" },
};

// 누산기: 클럭이 올라갈 때 값을 받거나(더할까=0) 더한다(더할까=1)
static const Step ACC_SCRIPT[] = {
    { {0,0,0}, {0}, 0, 0, nullptr },
    { {1,0,0}, {0}, 1, 0, "0 을 넣어 자리를 잡는다" },
    { {0,20,0}, {0}, 1, 0, "클럭이 안 올라갔으니 아직" },
    { {1,20,0}, {20}, 1, 0, "더할까가 꺼져 있으면 그냥 넣는다" },
    { {0,22,1}, {20}, 1, 0, "아직" },
    { {1,22,1}, {42}, 1, 0, "더할까가 켜져 있으면 더한다 (20+22)" },
    { {0,1,1},  {42}, 1, 0, "그대로" },
    { {1,1,1},  {43}, 1, 0, "또 더하면 43" },
    { {1,7,0},  {43}, 1, 0, "클럭이 이미 올라가 있으면 안 바뀐다" },
    { {0,7,0},  {43}, 1, 0, "내려가도 그대로" },
    { {1,7,0},  {7},  1, 0, "다시 올라갈 때 7 을 넣는다" },
};

// 컴퓨터가 돌릴 프로그램. 한 바이트가 명령 하나다.
//   윗 2비트 = 무엇을, 아랫 6비트 = 어떤 수로
//   1 = 넣기, 2 = 더하기, 0 = 멈춤
static const Val CPU_PROG[] = {
    (1u << 6) | 20,     // 누산기 = 20
    (2u << 6) | 22,     // 누산기 += 22   → 42
    0,                  // 멈춤
};

// 컴퓨터: 램에 넣어 둔 프로그램을 스스로 읽어 돈다.
// 프로그램(램에 미리 넣어 둔다):
//   0번: 1 (불러오기) → 5번 자리의 값을 누산기에
//   1번: 5
//   2번: 2 (더하기)   → 6번 자리의 값을 누산기에 더함
//   3번: 6
//   4번: 0 (멈춤)
//   5번: 20,  6번: 22   →  20 + 22 = 42
static const Step CPU_SCRIPT[] = {
    { {0,1}, {0},  0, 0, nullptr },             // 되돌리기를 켠 채로
    { {1,1}, {0},  0, 0, nullptr },             // 세는상자를 0 으로
    { {0,0}, {0},  0, 0, nullptr },             // 되돌리기 뗌 — 0번 명령을 보는 중
    { {1,0}, {20}, 1, 0, "첫 명령(넣기 20)이 돌면 누산기가 20" },
    { {0,0}, {20}, 1, 0, "그대로" },
    { {1,0}, {42}, 1, 0, "둘째 명령(더하기 22)이 돌면 42" },
    { {0,0}, {42}, 1, 0, "그대로" },
    { {1,0}, {42}, 1, 0, "셋째는 멈춤이라 안 바뀐다" },
    { {0,0}, {42}, 1, 0, "42 그대로" },
    { {1,0}, {42}, 1, 0, "계속 멈춰 있어야 한다" },
};

// 스위치묶음으로 값 만들기
static const Step SWB_SCRIPT[] = {
    { {0}, {181}, 1, 0, "스위치묶음을 181 로 맞춰 출력핀에 이어라" },
};

// 화면에 줄을 켜 보기
static const Step SCR_SCRIPT[] = {
    { {255, 0},  {255}, 1, 0, "윗줄을 다 켠다" },
    { {255, 15}, {255}, 1, 0, "가운뎃줄은 절반만" },
};

// 글자 찍기: 주소를 주면 그 줄의 그림을 내놓는 회로
// 램에 미리 넣어 둘 그림 (8줄, 각 줄이 8칸). 'ㄱ' 비슷한 모양.
static const Val GLYPH_PROG[] = {
    0xFE, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
};
static const Step GLYPH_SCRIPT[] = {
    { {0}, {0xFE}, 1, 0, "0번 줄" },
    { {1}, {0x02}, 1, 0, "1번 줄" },
    { {4}, {0x02}, 1, 0, "4번 줄" },
    { {7}, {0x02}, 1, 0, "7번 줄" },
};

static const Lesson LESSONS[] = {
    // ── 1부. 판에 익숙해지기 ──
    { "", "1. 이어 보기",
      "왼쪽 파란 것이 입력핀, 오른쪽 주황이 출력핀이다.\n"
      "입력핀은 눌러서 켜고 끌 수 있다.\n\n"
      "입력핀 오른쪽의 동그란 점을 잡고 끌어서\n"
      "출력핀 왼쪽 점에 놓아 보자.\n\n"
      "켜면 켜지고, 끄면 꺼진다.",
      1, 1, { "들어옴" }, { "나감" },
      { {0}, {1} }, "게이트 없이", false,
      "입력핀 오른쪽 가장자리에 작은 동그라미가 있다.\n그걸 눌러서 출력핀 왼쪽 동그라미까지 끌면 이어진다.",
      {}, {}, nullptr, 0, nullptr, 0 },

    { "", "2. 하나가 둘을",
      "한 출력에서 나온 선은 여러 곳에 이어도 된다.\n"
      "신호는 나눠 갖는 게 아니라 그대로 복사된다.\n\n"
      "입력 하나로 출력 둘을 같이 켜 보자.",
      1, 2, { "들어옴" }, { "왼쪽", "오른쪽" },
      { {0,0}, {1,1} }, "게이트 없이", false,
      "같은 동그라미에서 선을 두 번 끌면 된다.\n첫 선을 이은 뒤 그 동그라미를 다시 잡아라.",
      {}, {}, nullptr, 0, nullptr, 0 },

    // ── 2부. 게이트 하나씩 (직접 써 보며 익힌다) ──
    { "", "3. AND — 둘 다",
      "AND 는 두 입력이 모두 켜졌을 때만 켜진다.\n"
      "하나라도 꺼지면 꺼진다.\n\n"
      "'그리고' 라고 읽으면 된다.\n"
      "AND 를 하나 놓고 표대로 되는지 보자.",
      2, 1, { "A", "B" }, { "결과" },
      { {0}, {0}, {0}, {1} }, "AND", false,
      "왼쪽 목록에서 AND 를 고르고 판을 누르면 놓인다.\n왼쪽 두 점이 입력, 오른쪽 한 점이 출력이다.",
      {}, {}, nullptr, 0, nullptr, 0 },

    { "", "4. OR — 하나만이라도",
      "OR 은 둘 중 하나만 켜져도 켜진다.\n"
      "둘 다 꺼졌을 때만 꺼진다.\n\n"
      "'또는' 이다.",
      2, 1, { "A", "B" }, { "결과" },
      { {0}, {1}, {1}, {1} }, "OR", false,
      "AND 때와 똑같이 놓고 이으면 된다.\n표만 다르지 손은 같다.",
      {}, {}, nullptr, 0, nullptr, 0 },

    { "", "5. NOT — 뒤집기",
      "NOT 은 입력이 하나뿐이고 그것을 뒤집는다.\n"
      "켜지면 꺼지고, 꺼지면 켜진다.\n\n"
      "'아니다' 다.",
      1, 1, { "A" }, { "결과" },
      { {1}, {0} }, "NOT", false,
      "NOT 은 왼쪽 점이 하나뿐이다.\n입력핀 하나를 거기 이으면 끝난다.",
      {}, {}, nullptr, 0, nullptr, 0 },

    { "", "6. XOR — 다를 때만",
      "XOR 은 둘이 서로 다를 때만 켜진다.\n"
      "둘 다 켜져도 꺼진다는 게 OR 과 다르다.\n\n"
      "'둘 중 하나만' 이다. 나중에 덧셈에 쓰인다.",
      2, 1, { "A", "B" }, { "결과" },
      { {0}, {1}, {1}, {0} }, "XOR", false,
      "XOR 을 놓고 A 와 B 를 양쪽에 이어라.\nOR 과 달리 둘 다 켜지면 꺼지는지 눌러 보자.",
      {}, {}, nullptr, 0, nullptr, 0 },

    { "", "7. NAND — AND 의 반대",
      "NAND 는 AND 뒤에 NOT 을 붙인 것이다.\n"
      "둘 다 켜졌을 때만 꺼진다.\n\n"
      "이 게이트 하나로 다른 게이트를 전부 만들 수 있다.\n"
      "뒤에서 그걸 해 본다.",
      2, 1, { "A", "B" }, { "결과" },
      { {1}, {1}, {1}, {0} }, "NAND", false,
      "NAND 를 놓고 A, B 를 이어라.\n표를 보면 AND 를 위아래로 뒤집은 모양이다.",
      {}, {}, nullptr, 0, nullptr, 0 },

    { "", "8. NOR — OR 의 반대",
      "NOR 은 OR 뒤에 NOT 을 붙인 것이다.\n"
      "둘 다 꺼졌을 때만 켜진다.\n\n"
      "NOR 두 개를 서로 물리면 값을 기억하게 된다.",
      2, 1, { "A", "B" }, { "결과" },
      { {1}, {0}, {0}, {0} }, "NOR", false,
      "NOR 을 놓고 A, B 를 이어라.\n둘 다 꺼졌을 때만 켜지는지 확인하자.",
      {}, {}, nullptr, 0, nullptr, 0 },

    // ── 3부. 여러 개 이어 쓰기 ──
    { "", "9. 셋 다 켜졌을 때",
      "AND 는 입력이 둘뿐이다.\n"
      "셋을 보려면 어떻게 해야 할까?\n\n"
      "먼저 둘을 보고, 그 결과와 나머지 하나를 다시 본다.\n"
      "게이트의 출력은 다른 게이트의 입력이 될 수 있다.",
      3, 1, { "A", "B", "C" }, { "결과" },
      { {0}, {0}, {0}, {0}, {0}, {0}, {0}, {1} }, "AND", false,
      "AND 두 개를 쓴다.\n첫 AND 에 A 와 B,\n둘째 AND 에 '첫 AND 의 출력' 과 C.",
      {}, {}, nullptr, 0, nullptr, 0 },

    { "", "10. 셋 중 하나라도",
      "이번엔 OR 로 같은 것을 해 보자.\n"
      "셋 중 하나만 켜져도 켜진다.",
      3, 1, { "A", "B", "C" }, { "결과" },
      { {0}, {1}, {1}, {1}, {1}, {1}, {1}, {1} }, "OR", false,
      "9단계와 똑같은 모양인데 AND 대신 OR 을 쓴다.",
      {}, {}, nullptr, 0, nullptr, 0 },

    { "", "11. 다수결",
      "셋 중 둘 이상이 켜지면 켜진다.\n\n"
      "짝을 지어 보자 — A와B, B와C, A와C.\n"
      "그 중 하나라도 둘 다 켜졌으면 된다.",
      3, 1, { "A", "B", "C" }, { "결과" },
      { {0}, {0}, {0}, {1}, {0}, {1}, {1}, {1} }, "AND OR", false,
      "AND 셋으로 A와B, B와C, A와C 를 각각 본다.\n그 셋 중 하나라도 켜지면 되니까\nOR 둘로 묶는다.",
      {}, {}, nullptr, 0, nullptr, 0 },

    // ── 4부. NAND 하나로 다시 만들기 ──
    { "NOT짜기", "12. NAND 로 NOT",
      "여기서부터는 NAND 만 쓴다.\n\n"
      "NAND 는 둘 다 켜져야 꺼진다.\n"
      "두 입력에 같은 것을 넣으면 어떻게 될까?\n\n"
      "통과하면 이 회로가 부품이 되어 다음 단계에서 쓰인다.",
      1, 1, { "A" }, { "결과" },
      { {1}, {0} }, "NAND", true,
      "NAND 의 두 입력에 같은 것을 넣는다.\n입력핀의 출력 동그라미에서\n선을 두 번 끌면 된다.",
      {}, {}, nullptr, 0, nullptr, 0 },

    { "AND짜기", "13. NAND 로 AND",
      "NAND 는 AND 를 뒤집은 것이다.\n"
      "그러면 한 번 더 뒤집으면 AND 가 된다.\n\n"
      "앞 단계에서 만든 NOT짜기 를 쓸 수 있다.",
      2, 1, { "A", "B" }, { "결과" },
      { {0}, {0}, {0}, {1} }, "NAND NOT짜기", true,
      "NAND(A,B) 는 AND 를 뒤집은 값이다.\n그 결과를 NOT짜기 에 한 번 더 통과시켜라.",
      {}, {}, nullptr, 0, nullptr, 0 },

    { "OR짜기", "14. NAND 로 OR",
      "드모르간 법칙:\n"
      "  A 또는 B  =  (A아님 그리고 B아님) 아님\n\n"
      "두 입력을 각각 뒤집어서 NAND 에 넣어 보자.",
      2, 1, { "A", "B" }, { "결과" },
      { {0}, {1}, {1}, {1} }, "NAND NOT짜기", true,
      "A 와 B 를 각각 NOT짜기 에 통과시킨다.\n그 둘을 NAND 에 넣으면 OR 이 된다.",
      {}, {}, nullptr, 0, nullptr, 0 },

    { "XOR짜기", "15. NAND 로 XOR",
      "다를 때만 켜진다는 것은\n"
      "  '하나라도 켜졌고, 둘 다는 아니다'\n"
      "라는 뜻이다.\n\n"
      "앞의 것과 NAND 를 AND짜기 로 묶어 보자.",
      2, 1, { "A", "B" }, { "결과" },
      { {0}, {1}, {1}, {0} }, "NAND NOT짜기 AND짜기 OR짜기", true,
      "'하나라도 켜짐' 은 OR짜기(A,B),\n'둘 다는 아님' 은 NAND(A,B) 다.\n그 둘을 AND짜기 로 묶어라.",
      {}, {}, nullptr, 0, nullptr, 0 },

    // ── 5부. 덧셈 ──
    { "반가산기", "16. 한 자리 덧셈",
      "0+0=0,  0+1=1,  1+0=1,  1+1=10\n\n"
      "마지막이 두 자리다. 아랫자리를 합,\n"
      "윗자리를 자리올림이라고 한다.\n\n"
      "표를 잘 보면 합은 XOR, 올림은 AND 다.",
      2, 2, { "A", "B" }, { "합", "올림" },
      { {0,0}, {1,0}, {1,0}, {0,1} }, "", true,
      "합과 올림은 서로 상관없는 두 회로다.\n합 = XOR(A,B),  올림 = AND(A,B).\nA 와 B 에서 선을 두 번씩 끌면 된다.",
      {}, {}, nullptr, 0, nullptr, 0 },

    { "전가산기", "17. 자리올림까지 받기",
      "여러 자리를 더하려면 아랫자리에서 올라온\n"
      "자리올림도 같이 더해야 한다.\n\n"
      "입력 셋을 더해 합과 올림을 낸다.\n"
      "반가산기 두 개와 OR 하나면 된다.",
      3, 2, { "A", "B", "올림입력" }, { "합", "올림" },
      { {0,0}, {1,0}, {1,0}, {0,1}, {1,0}, {0,1}, {0,1}, {1,1} }, "", true,
      "반가산기 둘을 잇는다.\n첫째에 A,B → 둘째에 (첫째의 합) 과 올림입력.\n합은 둘째의 합.\n올림은 두 반가산기의 올림을 OR 로.",
      {}, {}, nullptr, 0, nullptr, 0 },

    // ── 6부. 기억하는 회로 ──
    { "SR래치", "18. 기억하기",
      "지금까지는 입력만 보면 답이 정해졌다.\n"
      "이제 지난 일을 기억하는 회로를 만든다.\n\n"
      "NOR 둘을 서로 물린다 — 한쪽 출력을 다른 쪽 입력에.\n\n"
      "셋을 잠깐 켜면 켜진 채로 남고,\n"
      "리셋을 잠깐 켜면 꺼진 채로 남는다.\n\n"
      "처음엔 어느 쪽인지 안 정해져서 깜빡일 수 있다.\n"
      "셋이나 리셋을 한 번 눌러 주면 자리를 잡는다.",
      2, 1, { "셋", "리셋" }, { "Q" },
      {}, "NOR", true,
      "NOR 두 개를 마주 보게 놓는다.\n첫째 NOR: 리셋 과 '둘째 출력'\n둘째 NOR: 셋 과 '첫째 출력'\nQ 는 첫째 NOR 에서 뽑는다.",
      {}, {}, LAT_SCRIPT, 6, nullptr, 0 },

    { "D래치", "19. 값을 잡아 두기",
      "셋·리셋을 따로 주는 건 불편하다.\n"
      "값 하나와 '지금 받아라' 하나면 좋겠다.\n\n"
      "받아라가 켜져 있을 때만 값이 통하고,\n"
      "꺼지면 그때 값을 그대로 붙잡게 만든다.\n\n"
      "SR래치 앞에 AND 둘을 붙이면 된다.",
      2, 1, { "값", "받아라" }, { "Q" },
      {}, "SR래치 AND NOT", true,
      "셋  = 값 그리고 받아라\n리셋 = (값 아님) 그리고 받아라\n이 둘을 SR래치에 넣는다.\n받아라가 꺼지면 둘 다 0 이라 그대로 있는다.",
      {}, {}, DLAT_SCRIPT, 7, nullptr, 0 },

    { "플립플롭", "20. 순간만 잡기",
      "D래치는 받아라가 켜져 있는 내내 값이 통한다.\n"
      "그래서 되먹임 회로에 쓰면 값이 계속 돌아 버린다.\n\n"
      "필요한 건 '클럭이 올라가는 그 순간' 만 잡는 것이다.\n\n"
      "D래치 둘을 이어 쓴다. 앞엣것은 클럭이 꺼져 있을 때 받고,\n"
      "뒤엣것은 켜져 있을 때 받는다. 그러면 넘어가는 값은\n"
      "올라가기 직전의 값 하나뿐이다.",
      2, 1, { "값", "클럭" }, { "Q" },
      {}, "D래치 NOT", true,
      "D래치 둘을 나란히.\n앞엣것 받아라 = 클럭 아님(NOT)\n뒤엣것 받아라 = 클럭\n앞엣것 Q → 뒤엣것 값\nQ 는 뒤엣것에서 뽑는다.",
      {}, {}, FF_SCRIPT, 8, nullptr, 0 },

    // ── 7부. 여덟 자리 다루기 ──
    { "", "21. 눌러서 넣기",
      "여기서부터는 8비트 값을 다룬다.\n"
      "그걸 스위치 여덟으로 만드는 건 번거롭다.\n\n"
      "**스위치묶음** 은 그 여덟을 한 부품으로 묶은 것이다.\n"
      "칸을 눌러 비트를 켜고 끄면 아래에 값이 뜬다.\n\n"
      "스위치묶음 하나를 놓고 181 을 만들어\n"
      "출력핀으로 내보내 보자.",
      1, 1, { "안씀" }, { "값" },
      {}, "스위치묶음", false,
      "스위치묶음을 놓고 폭이 8인지 본다(W 로 바꿈).\n181 = 10110101 이다.\n맨 아래 칸이 1의 자리.",
      {}, { 8 }, SWB_SCRIPT, 1, nullptr, 0 },

    { "레지스터", "22. 여덟 칸 기억",
      "플립플롭 하나는 1비트를 기억한다.\n"
      "여덟 개를 나란히 두면 한 바이트를 기억한다.\n\n"
      "굵은 선(8비트)을 풀음으로 여덟 가닥으로 풀고,\n"
      "각각 플립플롭에 넣은 뒤, 묶음으로 다시 묶는다.\n\n"
      "클럭은 여덟 개 모두에 같이 넣는다.",
      2, 1, { "값", "클럭" }, { "나온값" },
      {}, "플립플롭 묶음 풀음", true,
      "풀음(8비트) 하나, 플립플롭 여덟, 묶음(8비트) 하나.\n클럭 하나를 여덟 군데로 나눠 넣는다.\n한 점에서 선을 여러 번 끌면 된다.",
      { 8, 1 }, { 8 }, REG_SCRIPT, 8, nullptr, 0 },

    // ── 8부. 계산과 세기 ──
    { "덧셈기8", "23. 여덟 자리 덧셈",
      "전가산기 하나는 한 자리를 더한다.\n"
      "여덟 개를 이어 붙이면 한 바이트를 더한다.\n\n"
      "아랫자리의 자리올림을 윗자리의 올림입력에 넣는다.\n"
      "맨 아랫자리의 올림입력은 0,\n"
      "맨 윗자리에서 나온 올림이 넘침이 된다.",
      2, 2, { "A", "B" }, { "합", "넘침" },
      {}, "전가산기 묶음 풀음", true,
      "풀음 둘로 A 와 B 를 여덟 가닥씩 푼다.\n전가산기 여덟을 세로로 놓고\n앞의 올림을 뒤의 올림입력에 잇는다.\n합 여덟 개는 묶음으로 다시 묶는다.",
      { 8, 8 }, { 8, 1 }, ADD8_SCRIPT, 6, nullptr, 0 },

    { "세는상자", "24. 세기",
      "클럭이 올라갈 때마다 1씩 느는 상자다.\n"
      "다음에 실행할 명령이 어디 있는지 세는 데 쓴다.\n\n"
      "레지스터에 담긴 값에 1을 더해서\n"
      "그 결과를 다시 레지스터에 넣으면 된다.\n\n"
      "되돌리기를 켜면 0 으로 돌아가야 한다.",
      2, 1, { "클럭", "되돌리기" }, { "값" },
      {}, "", true,
      "레지스터 하나, 덧셈기8 하나.\n레지스터 출력 → 덧셈기 A\nB 에는 1 (스위치 하나를 묶음의 0번에)\n덧셈기 합 → 레지스터 값\n되돌리기일 땐 0 이 들어가게 막는다.",
      { 1, 1 }, { 8 }, CNT_SCRIPT, 11, nullptr, 0 },

    // ── 9부. 계산기 ──
    { "계산기", "25. 더하고 빼기",
      "덧셈기로 빼기도 한다.\n\n"
      "B 를 뒤집고 1 을 더하면 -B 가 된다(2의 보수).\n"
      "그러면 A + (-B) 가 A - B 다.\n\n"
      "빼기 신호를 B 의 각 비트와 XOR 하면 뒤집히고,\n"
      "그 신호를 1 로도 쓰면 +1 까지 한꺼번에 된다.",
      3, 1, { "A", "B", "빼기" }, { "결과" },
      {}, "덧셈기8 묶음 풀음 XOR", true,
      "풀음으로 B 를 여덟 가닥 푼다.\n각 가닥을 빼기와 XOR 한다.\n묶음으로 다시 묶어 덧셈기8 의 B 에.\n빼기가 1이면 +1 도 해야 하는데,\n덧셈기8 에 올림입력이 없으니 A 쪽에 1을 더해도 된다.",
      { 8, 8, 1 }, { 8 }, ALU_SCRIPT, 6, nullptr, 0 },

    { "해독기", "26. 명령 읽기",
      "명령은 그냥 수다. 0 이면 멈춤, 1 이면 불러오기, 2 면 더하기.\n\n"
      "그 수를 보고 알맞은 선 하나만 켜 주는 회로가 해독기다.\n\n"
      "2비트를 풀어서 AND 와 NOT 으로 짝을 맞춘다.\n"
      "0 = 둘 다 꺼짐, 1 = 아랫자리만, 2 = 윗자리만.",
      1, 3, { "명령" }, { "멈춤", "불러오기", "더하기" },
      {}, "풀음 NOT AND", true,
      "풀음(2비트)으로 명령을 두 가닥으로.\n멈춤   = (아랫자리 아님) 그리고 (윗자리 아님)\n불러오기 = 아랫자리 그리고 (윗자리 아님)\n더하기  = (아랫자리 아님) 그리고 윗자리",
      { 2 }, {}, DEC_SCRIPT, 4, nullptr, 0 },

    // ── 10부. 컴퓨터 ──
    { "누산기", "27. 쌓아 두는 곳",
      "계산한 값을 담아 두는 레지스터를 누산기라고 한다.\n\n"
      "더할까가 꺼져 있으면 값을 그냥 담고,\n"
      "켜져 있으면 담긴 값에 더한다.\n\n"
      "요령은 힌트에 있다.",
      3, 1, { "클럭", "값", "더할까" }, { "누산기" },
      {}, "레지스터 계산기 묶음 풀음 AND", true,
      "계산기 A 에 (누산기값 그리고 더할까) 를 넣는다.\n더할까가 0 이면 A 가 0 이라 0+값 = 값.\n값은 계산기 B 로, 결과는 레지스터로.\n레지스터 출력이 누산기다.",
      { 1, 8, 1 }, { 8 }, ACC_SCRIPT, 11, nullptr, 0 },

    { "컴퓨터", "28. 스스로 도는 것",
      "이제 다 모은다. 램의 명령을 스스로 읽어 실행한다.\n\n"
      "명령 한 바이트 =\n"
      "  윗 2비트(무엇을) + 아랫 6비트(어떤 수로)\n"
      "  1 = 넣기,  2 = 더하기,  0 = 멈춤\n\n"
      "세는상자가 주소를, 램이 명령을 내놓는다.\n\n"
      "램에는 검사할 때 프로그램이 들어간다.",
      2, 1, { "클럭", "되돌리기" }, { "누산기" },
      {}, "", true,
      "세는상자 → 램 주소 (6비트만 쓰면 된다)\n램 출력을 풀어서 윗 2비트 → 해독기,\n아랫 6비트 → 묶음 → 누산기의 값\n해독기의 '불러오기 또는 더하기' 를 누산기 클럭으로,\n'더하기' 를 누산기의 더할까로.",
      { 1, 1 }, { 8 }, CPU_SCRIPT, 10, CPU_PROG, 3 },

    // ── 11부. 보고 찍기 ──
    { "", "29. 화면에 찍기",
      "**화면** 은 n개의 m비트 입력을 n줄 × m칸 점으로 보여 준다.\n"
      "입력 하나가 한 줄이고, 비트 하나가 점 하나다.\n"
      "왼쪽이 윗자리다.\n\n"
      "E 로 줄 수를, W 로 칸 수를 바꾼다.\n\n"
      "8줄짜리 화면에 아무 줄이나 켜 보자 —\n"
      "0번 줄이 전부 켜지고 3번 줄이 절반이면 통과다.",
      2, 1, { "윗줄", "가운뎃줄" }, { "확인" },
      {}, "화면 스위치묶음", false,
      "화면을 놓고 E 로 8줄, W 로 8칸.\n윗줄을 0번 입력에, 가운뎃줄을 3번 입력에 잇는다.\n확인 출력핀에는 윗줄을 그대로 이으면 된다.",
      { 8, 8 }, { 8 }, SCR_SCRIPT, 2, nullptr, 0 },

    { "글자찍개", "30. 램으로 그림 그리기",
      "램에 그림을 한 줄씩 담아 두면 화면에 찍을 수 있다.\n\n"
      "줄 번호를 주소로 넣으면 그 줄의 그림이 나온다.\n"
      "그게 곧 화면 한 줄이다.\n\n"
      "채점할 때 램에 그림이 들어간다. 넣지 않아도 된다.",
      1, 1, { "줄번호" }, { "그림" },
      {}, "", true,
      "램의 주소에 줄번호를 잇고,\n램이 내놓는 값을 그림으로 그대로 내보낸다.\n주소는 8비트라 묶음으로 폭을 맞춰야 할 수도 있다.",
      { 8 }, { 8 }, GLYPH_SCRIPT, 4, GLYPH_PROG, 8 },
};

static const int LESSON_N = (int)(sizeof(LESSONS) / sizeof(LESSONS[0]));

// 단계를 묶은 '부'. 난이도는 부마다 하나로 매긴다 —
// 같은 부 안의 단계들은 어차피 비슷한 수준이라 따로 매길 게 없다.
// first 는 그 부가 시작하는 단계 번호(0부터). 끝은 다음 부의 first 다.
struct Part { const char* name; const char* what; int diff; int first; };
static const Part PARTS[] = {
    { "판에 익숙해지기",     "부품을 놓고 선을 잇는다",            1,  0 },
    { "게이트 하나씩",       "여섯 게이트를 하나씩 써 본다",       1,  2 },
    { "여러 개 이어 쓰기",   "게이트를 이어 붙여 답을 만든다",     2,  8 },
    { "NAND 하나로 다시",    "NAND 만으로 나머지를 되만든다",      3, 11 },
    { "덧셈",                "비트를 더하고 자리올림을 넘긴다",    3, 15 },
    { "기억하는 회로",       "되먹여서 지난 일을 붙잡아 둔다",     4, 17 },
    { "여덟 자리 다루기",    "선 여덟 가닥을 한 값으로 다룬다",    4, 20 },
    { "계산과 세기",         "한 바이트를 더하고 스스로 센다",     4, 22 },
    { "계산기",              "빼기와 명령 해독까지",               5, 24 },
    { "컴퓨터",              "램의 프로그램을 스스로 읽어 돈다",   5, 26 },
    { "보고 찍기",           "점 격자에 그림을 찍는다",            3, 28 },
};
static const int PART_N = (int)(sizeof(PARTS) / sizeof(PARTS[0]));
static const int DIFF_MAX = 5;

// 그 단계가 든 부
static int partOf(int lesson) {
    int p = 0;
    for (int i = 0; i < PART_N; ++i) if (PARTS[i].first <= lesson) p = i;
    return p;
}
// 그 부의 끝 (다음 부가 시작하는 자리)
static int partEnd(int p) { return p + 1 < PART_N ? PARTS[p + 1].first : LESSON_N; }

// 이 단계에서 쓸 수 있는 부품인가 (allow 가 빈 칸이면 다 된다)
static bool lessonAllows(const Lesson& L, const char* partName) {
    if (!L.allow || !*L.allow) return true;
    std::string a = L.allow, n = partName;
    size_t p = 0;
    while (p < a.size()) {
        size_t e = a.find(' ', p);
        if (e == std::string::npos) e = a.size();
        if (a.compare(p, e - p, n) == 0) return true;
        p = e + 1;
    }
    return false;
}

// 판에서 이름이 label 인 핀을 찾는다
static int findPin(int type, const char* label) {
    for (int i = 0; i < (int)world.comps.size(); ++i) {
        const Comp& c = world.comps[i];
        if (c.alive && c.type == type && c.label == label) return i;
    }
    return -1;
}

// 채점. 못 맞추면 어디서 틀렸는지 why 에 적어 준다.
static int lesInW(const Lesson& L, int i)  { return L.inW[i]  ? L.inW[i]  : 1; }
static int lesOutW(const Lesson& L, int i) { return L.outW[i] ? L.outW[i] : 1; }

// 출력핀에 들어온 값 (폭이 있으면 수 그대로)
static Val pinVal(int idx) {
    if (idx < 0 || idx >= (int)world.comps.size()) return 0;
    const Comp& c = world.comps[idx];
    return c.in.empty() ? 0 : c.in[0];
}

static bool gradeLesson(const Lesson& L, char* why, size_t whyN) {
    int in[LES_IN_MAX], out[LES_OUT_MAX];
    for (int i = 0; i < L.nIn; ++i) {
        in[i] = findPin(PIN_IN, L.inName[i]);
        if (in[i] < 0) { std::snprintf(why, whyN, "'%s' 이름의 입력핀이 없다", L.inName[i]); return false; }
    }
    for (int i = 0; i < L.nOut; ++i) {
        out[i] = findPin(PIN_OUT, L.outName[i]);
        if (out[i] < 0) { std::snprintf(why, whyN, "'%s' 이름의 출력핀이 없다", L.outName[i]); return false; }
    }
    // 프로그램이 있는 단계는 판에 놓인 램에 미리 넣어 준다 (맨 처음 램 하나)
    if (L.program && L.nProg > 0) {
        int ramAt = -1;
        for (int i = 0; i < (int)world.comps.size(); ++i)
            if (world.comps[i].alive && world.comps[i].type == RAM) { ramAt = i; break; }
        if (ramAt < 0) { std::snprintf(why, whyN, "판에 램이 없다"); return false; }
        Comp& r = world.comps[ramAt];
        r.mem.assign(RAM_WORDS, 0);
        for (int i = 0; i < L.nProg && i < RAM_WORDS; ++i)
            r.mem[i] = maskTo(L.program[i], RAM_DATAW);
    }

    // ── 값을 차례로 넣어 보는 단계 (기억하는 회로) ──
    if (L.script && L.nStep > 0) {
        // 처음 상태를 맞춰 둔다 — 앞 검사에서 남은 값이 섞이면 안 된다
        for (int i = 0; i < L.nIn; ++i) {
            auto& o = world.comps[in[i]].out;
            if (!o.empty()) o[0] = 0;
        }
        for (int k = 0; k < 40; ++k) tickSub(world);

        for (int st = 0; st < L.nStep; ++st) {
            const Step& sp = L.script[st];
            for (int i = 0; i < L.nIn; ++i) {
                auto& o = world.comps[in[i]].out;
                if (!o.empty()) o[0] = maskTo(sp.in[i], lesInW(L, i));
            }
            int tk = sp.ticks ? sp.ticks : 120;   // 여덟 자리 이어지는 데 시간이 걸린다
            for (int k = 0; k < tk; ++k) tickSub(world);

            for (int i = 0; i < L.nOut; ++i) {
                if (!(sp.check & (1u << i))) continue;
                Val got  = maskTo(pinVal(out[i]), lesOutW(L, i));
                Val want = maskTo(sp.want[i], lesOutW(L, i));
                if (got != want) {
                    char inbuf[96] = "";
                    for (int j = 0; j < L.nIn; ++j) {
                        char one[28];
                        std::snprintf(one, sizeof(one), "%s%s=%d", j ? ", " : "",
                                      L.inName[j], (int)maskTo(sp.in[j], lesInW(L, j)));
                        std::strncat(inbuf, one, sizeof(inbuf) - std::strlen(inbuf) - 1);
                    }
                    std::snprintf(why, whyN, "%d번째: %s 일 때 %s 가 %d (%d 이어야 함)%s%s",
                                  st + 1, inbuf, L.outName[i], (int)got, (int)want,
                                  sp.note ? " — " : "", sp.note ? sp.note : "");
                    return false;
                }
            }
        }
        std::snprintf(why, whyN, "통과");
        return true;
    }

    int cases = 1 << L.nIn;
    for (int t = 0; t < cases; ++t) {
        for (int i = 0; i < L.nIn; ++i) {
            auto& o = world.comps[in[i]].out;
            if (!o.empty()) o[0] = (t >> i) & 1;
        }
        for (int k = 0; k < 80; ++k) tickSub(world);      // 값이 자리잡을 때까지
        for (int i = 0; i < L.nOut; ++i) {
            bool got = lit(world.comps[out[i]]);
            if (got != (bool)L.want[t][i]) {
                char inbuf[64] = "";
                for (int j = 0; j < L.nIn; ++j) {
                    char one[24];
                    std::snprintf(one, sizeof(one), "%s%s=%d", j ? ", " : "", L.inName[j], (t >> j) & 1);
                    std::strncat(inbuf, one, sizeof(inbuf) - std::strlen(inbuf) - 1);
                }
                std::snprintf(why, whyN, "%s 일 때 %s 가 %d (%d 이어야 함)",
                              inbuf, L.outName[i], (int)got, (int)L.want[t][i]);
                return false;
            }
        }
    }
    std::snprintf(why, whyN, "통과");
    return true;
}

// 단계를 시작한다. 판을 비우고 필요한 핀을 미리 놓아 준다 —
// 이름이 틀리면 채점을 못 하므로 손으로 적게 두면 걸리적거리기만 한다.
static void setupLesson(int at) {
    const Lesson& L = LESSONS[std::clamp(at, 0, LESSON_N - 1)];
    world = SubSim{};
    // 폭 있는 입력핀은 칸이 여럿이라 키가 크다. 겹치지 않게 키만큼 띄운다.
    int y = 60;
    for (int i = 0; i < L.nIn; ++i) {
        int c = addComp(world, PIN_IN, 90, y);
        world.comps[c].label = L.inName[i];
        world.comps[c].aux = lesInW(L, i);
        resizePorts(world.comps[c]);
        y += compH(world.comps[c]) + 40;
    }
    y = 80;
    for (int i = 0; i < L.nOut; ++i) {
        int c = addComp(world, PIN_OUT, 620, y);
        world.comps[c].label = L.outName[i];
        world.comps[c].aux = lesOutW(L, i);
        resizePorts(world.comps[c]);
        y += compH(world.comps[c]) + 40;
    }
    viewZoom = 1.0f; viewX = 0; viewY = 0;
}

// 통과했다. 지금 판을 그 이름의 칩으로 만들어 다음 단계에서 쓰게 한다.
static void bankLesson(const Lesson& L) {
    if (!L.bank || !L.name || !*L.name) return;      // 이해만 하는 단계는 안 남긴다
    for (const auto& c : chips) if (c.alive && c.name == L.name) return;   // 이미 있다
    std::vector<int> all;
    for (int i = 0; i < (int)world.comps.size(); ++i)
        if (world.comps[i].alive) all.push_back(i);
    if (all.empty()) return;
    // createChip 은 묶은 부품을 판에서 치우고 상자 하나로 바꾼다.
    // 여기서는 칩만 얻고 판은 풀던 그대로 두고 싶으니 판만 도로 돌려놓는다.
    SubSim keep = deepCopy(world);
    createChip(all, L.name);
    world = keep;                    // chips 에 새로 생긴 칩은 그대로 남는다
}


// 학습 모드에서는 그 단계에 허락된 부품만 쓸 수 있다.
// 손·핀·스위치·전구는 늘 된다 — 그게 없으면 아무것도 못 짠다.
static bool toolEnabled(int i) {
    // 고치는 중인 칩은 못 놓는다 — 자기 안에 자기가 들어가면 끝없이 깊어진다
    { int c = toolChip(i); if (c >= 0 && editingChip(c)) return false; }
    if (screen != SC_LEARN) return true;
    if (i == 0) return true;
    const Lesson& L = LESSONS[std::clamp(lessonAt, 0, LESSON_N - 1)];
    if (!L.allow || !*L.allow) return true;
    if (i <= TYPE_N) {
        int t = i - 1;
        if (isPin(t) || t == SWITCH || t == LAMP) return true;
        return lessonAllows(L, TYPES[t].name);
    }
    int c = toolChip(i);
    return c >= 0 && lessonAllows(L, chips[c].name.c_str());
}

// 판이 그려지는 가로 범위. Tab 으로 판들을 숨기면 화면 전체가 된다.
static int canvasL() { return uiOn ? PANEL_W : 0; }
static int canvasR() { return WIN_W - ((uiOn && screen == SC_LEARN) ? LES_W : 0); }

// UTF-8 한 글자 앞으로 물러난 자리 (이어지는 바이트 한가운데를 안 자르게)
static size_t backOneChar(const std::string& t, size_t i) {
    if (i == 0) return 0;
    --i;
    while (i > 0 && (t[i] & 0xC0) == 0x80) --i;
    return i;
}

// 빈칸 없이 긴 덩어리를 글자 단위로 잘라 자리에 맞춘다
static std::vector<std::string> hardSplit(const std::string& word, int px, int maxW) {
    std::vector<std::string> out;
    std::string rest = word;
    while (textWidth(px, rest.c_str()) > maxW) {
        size_t cut = rest.size();
        // 들어갈 때까지 한 글자씩 줄인다
        while (cut > 0) {
            size_t nc = backOneChar(rest, cut);
            if (nc == 0) break;
            if (textWidth(px, rest.substr(0, nc).c_str()) <= maxW) { cut = nc; break; }
            cut = nc;
        }
        if (cut == 0 || cut >= rest.size()) break;      // 한 글자도 못 넣는 자리면 포기
        out.push_back(rest.substr(0, cut));
        rest = rest.substr(cut);
    }
    out.push_back(rest);
    return out;
}

// 글 한 줄을 픽셀 너비에 맞게 잘라 여러 줄로 만든다. 빈칸에서 끊고,
// 빈칸 없이 긴 말은 글자 단위로 끊는다.
static std::vector<std::string> wrapText(const std::string& line, int px, int maxW) {
    std::vector<std::string> out;
    if (line.empty()) { out.push_back(""); return out; }

    std::string cur;
    size_t i = 0;
    auto flush = [&] { if (!cur.empty()) { out.push_back(cur); cur.clear(); } };

    while (i < line.size()) {
        size_t j = line.find(' ', i);
        std::string word = (j == std::string::npos) ? line.substr(i) : line.substr(i, j - i + 1);
        i = (j == std::string::npos) ? line.size() : j + 1;

        if (textWidth(px, word.c_str()) > maxW) {       // 덩어리 자체가 넘친다
            flush();
            std::vector<std::string> ps = hardSplit(word, px, maxW);
            for (size_t k = 0; k + 1 < ps.size(); ++k) out.push_back(ps[k]);
            cur = ps.back();
            continue;
        }
        if (!cur.empty() && textWidth(px, (cur + word).c_str()) > maxW) flush();
        cur += word;
    }
    if (!cur.empty() || out.empty()) out.push_back(cur);
    return out;
}

// 힌트를 픽셀 너비에 맞춰 실제로 그릴 줄들로 편다
static std::vector<std::string> hintLines(const char* hint, int px, int maxW) {
    std::vector<std::string> out;
    std::string h = hint ? hint : "";
    size_t q = 0;
    while (q <= h.size()) {
        size_t e = h.find('\n', q);
        if (e == std::string::npos) e = h.size();
        for (auto& l : wrapText(h.substr(q, e - q), px, maxW)) out.push_back(l);
        if (e == h.size()) break;
        q = e + 1;
    }
    return out;
}

// 설명판 글자 크기 고르기. 창이 작으면 줄여서 단추를 안 밀어내고,
// 큰 창에서는 큼직하게 둔다. 그리는 쪽과 검사가 같은 값을 보게 따로 뺐다.
// 난이도를 작은 네모 다섯으로. 글꼴에 없는 기호를 쓰면 두부가 되니 직접 그린다.
static void drawDiff(SDL_Renderer* ren, int x, int y, int d, bool dim = false) {
    int w = S(9), h = S(9), gap = S(4);
    for (int i = 0; i < DIFF_MAX; ++i) {
        Rect r{ x + i * (w + gap), y, w, h };
        if (i < d) fillRect(ren, r, dim ? 0x46536A : 0x8FB4D8);
        else       frameRect(ren, r, dim ? 0x2E3440 : 0x3E4554);
    }
}
static int diffW() { return DIFF_MAX * S(9) + (DIFF_MAX - 1) * S(4); }

// 설명판 맨 위 '부' 머리말. 자리가 모자라면 소개 줄부터 접고, 더 모자라면 통째로 접는다.
// (접혀도 같은 것을 단계 목록 화면에서 볼 수 있다.)
static int lesHdrLv(int ts) { return ts >= 12 ? 2 : ts >= 10 ? 1 : 0; }
static int lesHdrH(int ts) {
    int lv = lesHdrLv(ts);
    if (lv == 0) return 0;
    return S(ts + 4) + (lv == 2 ? S(ts + 2) : 0) + S(10);
}
// 그 머리말을 누르면 단계 목록이 열린다
static Rect lesHdr(int ts) { return { WIN_W - LES_W, 0, LES_W, S(14) + lesHdrH(ts) }; }

struct LesFit { int ts, tl, rh, hs, need, room; };

static LesFit lessonFit(const Lesson& L, int winH) {
    int lines = 1;
    for (const char* c = L.text; *c; ++c) if (*c == '\n') ++lines;
    int rows = (L.script && L.nStep > 0) ? L.nStep : (1 << L.nIn);
    bool hasAllow = (L.allow && *L.allow);

    // ts·hs 는 1배 기준 글자 크기, tl·rh 는 화면에서의 줄 높이다.
    // 대본 단계는 진리표보다 칸이 많다. 표 줄을 좀 더 좁게 잡는다.
    bool tight = (L.script && L.nStep > 6);
    LesFit f{ 15, 0, 0, 15, 0, (winH - S(178)) - S(16) };
    for (;;) {
        f.tl = S(f.ts + 6);
        f.rh = S(f.ts + (tight ? 1 : 5));
        f.need = lesHdrH(f.ts)
               + S(34) + lines * f.tl + S(12) + (S(f.hs) + S(8)) + (f.rh + S(3)) + rows * f.rh
               + (hasAllow ? S(40) : 0);
        if (f.need <= f.room || f.ts <= 9) break;
        --f.ts; if (f.hs > 11) --f.hs;
    }
    return f;
}

static void drawLessonPanel(SDL_Renderer* ren, const char* msg, bool msgOK, int msgLeft) {
    const Lesson& L = LESSONS[std::clamp(lessonAt, 0, LESSON_N - 1)];
    Rect p = lessonPanel();
    fillRect(ren, p, 0x1B1E26);
    fillRect(ren, { p.x, 0, 1, WIN_H }, COL_LINE);

    // 설명 줄 수를 먼저 센다
    std::string t = L.text;
    int lines = 1;
    for (char c : t) if (c == '\n') ++lines;
    int rows = 1 << L.nIn;
    bool hasAllow = (L.allow && *L.allow);

    LesFit f = lessonFit(L, WIN_H);
    int ts = f.ts, tl = f.tl, rh = f.rh, hs = f.hs;
    (void)lines; (void)rows; (void)hasAllow;

    int y = S(14);
    if (lesHdrLv(ts)) {   // 부 머리말 — 어느 묶음이고 얼마나 어려운지. 누르면 단계 목록.
        int pi = partOf(lessonAt);
        const Part& P = PARTS[pi];
        char h[128];
        std::snprintf(h, sizeof(h), "%d부 · %s", pi + 1, P.name);
        drawText(ren, p.x + S(16), y, ts - 1, 0x8FB4D8, h);
        drawDiff(ren, p.x + LES_W - S(16) - diffW(), y + S(3), P.diff);
        y += S(ts + 4);
        if (lesHdrLv(ts) == 2) {
            drawText(ren, p.x + S(16), y, ts - 4, 0x6E7484, P.what);
            std::snprintf(h, sizeof(h), "%d / %d · 목록", lessonAt + 1, LESSON_N);
            drawText(ren, p.x + LES_W - S(16) - textWidth(ts - 4, h), y, ts - 4, 0x8A90A0, h);
            y += S(ts + 2);
        }
        setCol(ren, 0x2E3440);
        SDL_RenderDrawLine(ren, p.x + S(16), y + S(4), p.x + LES_W - S(16), y + S(4));
        y += S(10);
    }
    drawText(ren, p.x + S(16), y, ts + 5, 0xE8ECF4, L.title); y += S(34);

    size_t s0 = 0;
    while (s0 <= t.size()) {
        size_t e = t.find('\n', s0);
        if (e == std::string::npos) e = t.size();
        std::string line = t.substr(s0, e - s0);
        if (!line.empty()) drawText(ren, p.x + S(16), y, ts, 0xA6ACBA, line.c_str());
        y += tl;
        if (e == t.size()) break;
        s0 = e + 1;
    }

    y += S(12);
    drawText(ren, p.x + S(16), y, hs, 0xC8CEDC,
             (L.script && L.nStep > 0) ? "차례대로 이렇게" : "이렇게 돌아야 한다");
    y += S(hs) + S(8);

    // 진리표
    int nCol = L.nIn + L.nOut;
    int colW = std::min(S(46), (LES_W - S(32)) / std::max(1, nCol));
    int x0 = p.x + S(16);
    for (int i = 0; i < L.nIn; ++i)
        drawTextC(ren, x0 + colW/2 + i*colW, y, ts - 2, 0x8FB4D8, L.inName[i]);
    for (int i = 0; i < L.nOut; ++i)
        drawTextC(ren, x0 + colW/2 + (L.nIn + i)*colW, y, ts - 2, 0xD8A98F, L.outName[i]);
    y += rh + S(3);
    setCol(ren, 0x3A4050);
    SDL_RenderDrawLine(ren, x0, y - S(5), x0 + nCol*colW, y - S(5));
    SDL_RenderDrawLine(ren, x0 + L.nIn*colW - S(2), y - rh - S(6),
                            x0 + L.nIn*colW - S(2), y + rows*rh);

    bool useScript = (L.script && L.nStep > 0);
    for (int t2 = 0; t2 < rows; ++t2) {
        char b[8];
        for (int i = 0; i < L.nIn; ++i) {
            if (useScript) std::snprintf(b, sizeof(b), "%d",
                              (int)maskTo(L.script[t2].in[i], lesInW(L, i)));
            else           std::snprintf(b, sizeof(b), "%d", (t2 >> i) & 1);
            drawTextC(ren, x0 + colW/2 + i*colW, y, ts, 0x8A90A0, b);
        }
        for (int i = 0; i < L.nOut; ++i) {
            if (useScript) {
                if (L.script[t2].check & (1u << i))
                    std::snprintf(b, sizeof(b), "%d",
                                  (int)maskTo(L.script[t2].want[i], lesOutW(L, i)));
                else std::snprintf(b, sizeof(b), "·");       // 이 칸은 안 본다
            } else std::snprintf(b, sizeof(b), "%d", L.want[t2][i]);
            drawTextC(ren, x0 + colW/2 + (L.nIn + i)*colW, y, ts, 0xE8ECF4, b);
        }
        y += rh;
    }

    if (hasAllow) {
        y += S(14);
        drawText(ren, p.x + S(16), y, ts - 3, 0x6E7484, "쓸 수 있는 것:"); y += S(ts + 3);
        drawText(ren, p.x + S(16), y, ts - 1, 0x9AA0AE, L.allow);
    }

    // 채점 결과
    if (msgLeft > 0 && msg && *msg) {
        Rect m{ p.x + S(14), WIN_H - S(178), LES_W - S(28), S(62) };
        fillRect(ren, m, msgOK ? 0x1E3A2A : 0x3A2028);
        frameRect(ren, m, msgOK ? 0x4A9E6A : 0x9E4A5A);
        uint32_t fg = msgOK ? 0xA8E8C0 : 0xE8B0B8;
        std::vector<std::string> ml = hintLines(msg, 13, m.w - S(20));
        int my2 = m.y + (m.h - (int)ml.size() * S(19)) / 2;
        for (auto& l : ml) { drawText(ren, m.x + S(10), my2, 13, fg, l.c_str()); my2 += S(19); }
    }

    // 단계 넘기기 · 힌트
    Rect pv = btnPrevLes(), nx = btnNextLes(), hb = btnHint();
    bool canPrev = lessonAt > 0, canNext = lessonAt < LESSON_N - 1 && lessonAt < lessonDone;
    fillRect(ren, pv, canPrev ? 0x252A36 : 0x1E212A);
    int by = pv.y + (pv.h - S(14)) / 2;
    drawTextC(ren, pv.x + pv.w/2, by, 14, canPrev ? 0xC0C6D4 : 0x4A5060, "◀ 앞");
    fillRect(ren, hb, hintOn ? 0x4A421E : 0x2E2A1C);
    frameRect(ren, hb, hintOn ? 0x9E8A4A : 0x46402C);
    drawTextC(ren, hb.x + hb.w/2, by, 14, hintOn ? 0xEEDDA0 : 0xB0A67A,
              hintOn ? "힌트 닫기" : "힌트 (H)");
    fillRect(ren, nx, canNext ? 0x252A36 : 0x1E212A);
    drawTextC(ren, nx.x + nx.w/2, by, 14, canNext ? 0xC0C6D4 : 0x4A5060, "뒤 ▶");

    Rect g = btnGrade();
    fillRect(ren, g, 0x2C4A6A);
    frameRect(ren, g, 0x5A86BA);
    drawTextC(ren, g.x + g.w/2, g.y + (g.h - S(17))/2, 17, 0xD8E8FF, "채점하기  (Enter)");

}

// ─────────────────────────────────────────────────────────────
// 나갈까 묻는 창
// ─────────────────────────────────────────────────────────────

static Rect confirmBox()  { return { WIN_W/2 - S(210), WIN_H/2 - S(74), S(420), S(148) }; }
static Rect confirmYes()  { Rect b = confirmBox(); return { b.x + S(22), b.y + S(92), S(178), S(38) }; }
static Rect confirmNo()   { Rect b = confirmBox(); return { b.x + S(220), b.y + S(92), S(178), S(38) }; }

static void drawConfirm(SDL_Renderer* ren, int hover) {
    fillRect(ren, { 0, 0, WIN_W, WIN_H }, 0x000000, 160);
    Rect b = confirmBox();
    fillRect(ren, b, 0x232A36);
    frameRect(ren, b, 0x6C86C8);
    drawTextC(ren, b.x + b.w/2, b.y + S(22), 20, 0xE8ECF4, "첫 화면으로 나갈까?");
    drawTextC(ren, b.x + b.w/2, b.y + S(56), 13, 0x8A90A0, "하던 것은 저장된다. 다시 들어오면 그대로다.");

    Rect y = confirmYes(), n = confirmNo();
    fillRect(ren, y, hover == 0 ? 0x4A3038 : 0x33262C);
    frameRect(ren, y, hover == 0 ? 0xB06070 : 0x5A4048);
    drawTextC(ren, y.x + y.w/2, y.y + (y.h - S(16))/2, 16, 0xE8B8C0, "나가기  (Enter)");
    fillRect(ren, n, hover == 1 ? 0x2C4A6A : 0x263140);
    frameRect(ren, n, hover == 1 ? 0x5A86BA : 0x3A4A5E);
    drawTextC(ren, n.x + n.w/2, n.y + (n.h - S(16))/2, 16, 0xC8DCF4, "계속하기  (Esc)");
}

// 힌트는 설명판 안이 아니라 판 위에 띄운다.
// 설명이 긴 단계에서는 설명판에 자리가 없고, 여기가 넓어서 글자도 크게 넣을 수 있다.
static void drawHint(SDL_Renderer* ren) {
    const Lesson& L = LESSONS[std::clamp(lessonAt, 0, LESSON_N - 1)];
    if (!L.hint || !*L.hint) return;

    int padX = S(20), padY = S(16), lh = S(22), ts = 15;
    int maxW = canvasR() - canvasL() - S(24) - padX * 2;
    std::vector<std::string> ls = hintLines(L.hint, ts, maxW);

    int wide = 0;
    for (auto& l : ls) wide = std::max(wide, textWidth(ts, l.c_str()));
    int w  = std::min(wide + padX * 2, canvasR() - canvasL() - S(24));
    int hh = (int)ls.size() * lh + padY * 2 + S(20);
    Rect b{ (canvasL() + canvasR())/2 - w/2, WIN_H - hh - S(18), w, hh };

    fillRect(ren, b, 0x2A2718, 244);
    frameRect(ren, b, 0x9E8A4A);
    fillRect(ren, { b.x, b.y, b.w, S(3) }, 0xC8A84A);
    drawText(ren, b.x + padX, b.y + padY - S(4), 13, 0xC8A84A, "힌트");
    const char* close = "H 로 닫기";
    drawText(ren, b.x + b.w - textWidth(11, close) - padX, b.y + padY - S(2), 11, 0x8A7A4A, close);

    int y = b.y + padY + S(18);
    for (auto& l : ls) { drawText(ren, b.x + padX, y, ts, 0xEEE0B0, l.c_str()); y += lh; }
}

static void drawNaming(SDL_Renderer* ren, const std::string& name, int frame,
                       const char* title, const char* hint) {
    fillRect(ren, { 0, 0, WIN_W, WIN_H }, 0x000000, 150);
    Rect box{ WIN_W/2 - S(220), WIN_H/2 - S(64), S(440), S(128) };
    fillRect(ren, box, 0x232A36);
    frameRect(ren, box, 0x6C86C8);
    drawText(ren, box.x + S(20), box.y + S(16), 15, COL_DIM, title);
    std::string shown = name + ((frame / 30) % 2 ? "_" : " ");
    drawText(ren, box.x + S(20), box.y + S(48), 22, COL_TEXT, shown.c_str());
    drawText(ren, box.x + S(20), box.y + S(98), 12, COL_DIM, hint);
}

// 마우스가 올라간 포트의 이름을 말풍선으로 보여 준다
static void drawTip(SDL_Renderer* ren, int sx, int sy, const char* text) {
    int tw = textWidth(13, text);
    Rect b{ sx - tw/2 - S(10), sy - S(38), tw + S(20), S(28) };
    b.x = std::clamp(b.x, canvasL() + S(4), WIN_W - b.w - S(4));
    b.y = std::max(S(4), b.y);
    fillRect(ren, b, 0x2A3040, 240);
    frameRect(ren, b, 0x5A6480, 240);
    drawText(ren, b.x + S(10), b.y + (b.h - S(13))/2, 13, 0xE8ECF4, text);
}

// ─────────────────────────────────────────────────────────────
// 메인
// ─────────────────────────────────────────────────────────────

static void seedDemo() {
    int s = addComp(SWITCH, 120, 300);
    int l = addComp(LAMP,   360, 300);
    addWire(s, 0, l, 0);
}

// 판 배경의 눈금. 확대·이동한 게 눈에 보이게 한다.
static void drawGrid(SDL_Renderer* ren) {
    int step = 40;
    while (step * viewZoom < 18) step *= 2;      // 너무 줄면 성기게
    while (step * viewZoom > 90) step /= 2;
    if (step < 5) step = 5;
    setCol(ren, 0x1E2029);
    int x0 = (int)std::floor(viewX / step) * step;
    for (int wx = x0; ; wx += step) {
        int sx = w2sX((float)wx);
        if (sx > canvasR()) break;
        if (sx >= canvasL()) SDL_RenderDrawLine(ren, sx, 0, sx, WIN_H);
    }
    int y0 = (int)std::floor(viewY / step) * step;
    for (int wy = y0; ; wy += step) {
        int sy = w2sY((float)wy);
        if (sy > WIN_H) break;
        if (sy >= 0) SDL_RenderDrawLine(ren, canvasL(), sy, canvasR(), sy);
    }
}

// ─────────────────────────────────────────────────────────────
// 그리기
// ─────────────────────────────────────────────────────────────

static bool inSel(const std::vector<int>& sel, int i) {
    for (int s : sel) if (s == i) return true;
    return false;
}

// 판 좌표 상자를 화면 상자로
static Rect toScreen(int wx, int wy, int ww, int wh) {
    int sx = w2sX((float)wx), sy = w2sY((float)wy);
    return { sx, sy, std::max(1, w2sLen(ww)), std::max(1, w2sLen(wh)) };
}

// 굵은 선(여러 비트) 포트는 네모로, 한 비트짜리는 동그라미로 그린다.
// 색도 선이랑 맞춘다 — 굵은 쪽은 파랑끼, 가는 쪽은 초록.
static void drawPort(SDL_Renderer* ren, int sx, int sy, int r, int width, bool on) {
    if (width <= 1) {
        fillCircle(ren, sx, sy, r, on ? COL_ON : COL_OFF);
        return;
    }
    uint32_t col = on ? 0x6AC8E0 : 0x3A4A54;
    int h = r + std::max(1, r / 3);                 // 네모는 조금 크게
    fillRect(ren, { sx - h, sy - h, h * 2, h * 2 }, col);
    frameRect(ren, { sx - h, sy - h, h * 2, h * 2 }, shade(col, on ? 60 : 30));
    // 가운데에 가로줄 하나 — '여러 가닥이 묶여 있다' 는 표시
    if (r >= 4) fillRect(ren, { sx - h + 2, sy - 1, h * 2 - 4, std::max(1, r / 3) },
                         shade(col, on ? -70 : -25));
}

static void drawComp(SDL_Renderer* ren, int idx, const std::vector<int>& sel) {
    const Comp& c = world.comps[idx];
    int w = compW(c), h = compH(c);
    Rect box = toScreen(c.x, c.y, w, h);
    // 화면 밖이면 건너뛴다 (많이 놓아도 안 느려지게)
    if (box.x + box.w < PANEL_W || box.x > WIN_W || box.y + box.h < 0 || box.y > WIN_H) return;

    uint32_t base = compColor(c);
    bool on = lit(c);
    int   ts = std::max(7, (int)(15 * viewZoom));     // 글자 크기도 같이 줄고 는다
    int   cx = box.x + box.w / 2;

    if (isBitBank(c)) {
        // 겉모습이 스위치 n개다. 칸을 눌러 그 비트를 켜고 끈다.
        fillRect(ren, box, shade(base, -34));
        frameRect(ren, box, shade(base, 40));
        // 핀이면 이름을 위에 적어 준다 (어느 입력인지 알아야 하니까)
        if (c.type == PIN_IN)
            drawTextC(ren, cx, box.y + w2sLen(3), 13, 0xC8DCF4, compName(c));
        int wd = widthOf(c);
        Val v = c.out.empty() ? 0 : c.out[0];
        int pad = std::max(2, w2sLen(4));
        int top = (c.type == PIN_IN) ? w2sLen(18) : 0;
        int rowH = (box.h - w2sLen(20) - pad - top) / std::max(1, wd);
        for (int i = 0; i < wd; ++i) {
            bool on = (v >> i) & 1;                     // 0번이 맨 아래
            Rect r{ box.x + pad, box.y + box.h - w2sLen(20) - (i + 1) * rowH,
                    box.w - pad * 2, std::max(2, rowH - 1) };
            fillRect(ren, r, on ? blend(base, COL_ON, 170) : 0x252A30);
            if (rowH >= w2sLen(9)) {
                char d[2] = { (char)('0' + (on ? 1 : 0)), 0 };
                drawTextC(ren, r.x + r.w/2, r.y + (r.h - S(11))/2, 11,
                          on ? 0x0E140F : 0x6C7280, d);
            }
        }
        char b2[24]; std::snprintf(b2, sizeof(b2), "%d", (int)v);
        drawTextC(ren, cx, box.y + box.h - w2sLen(18), std::max(8, (int)(12 * viewZoom)),
                  0xC8E8D0, b2);
    } else if (isPin(c.type)) {
        // 핀은 제 이름이 곧 얼굴이다. 칩의 어느 포트가 될지 알아보게 테두리를 굵게.
        uint32_t col = on ? blend(base, COL_ON, 120) : shade(base, -34);
        fillRect(ren, box, col);
        frameRect(ren, box, on ? COL_ON : shade(base, 50));
        frameRect(ren, { box.x+1, box.y+1, box.w-2, box.h-2 }, shade(base, on ? 60 : 10));
        drawTextC(ren, cx, box.y + box.h/2 - ts*3/5, ts,
                  on ? 0x0E140F : COL_TEXT, compName(c));
        if (viewZoom > 0.55f) {
            char sub[32];
            int wd = widthOf(c);
            if (wd > 1) std::snprintf(sub, sizeof(sub), "%d비트  %d", wd, (int)shownVal(c));
            else        std::snprintf(sub, sizeof(sub), "%s", c.type == PIN_IN ? "입력핀" : "출력핀");
            drawTextC(ren, cx, box.y + box.h - w2sLen(15), std::max(7, (int)(10 * viewZoom)),
                      on ? 0x0E140F : shade(base, 70), sub);
        }
    } else if (c.type == CLOCK) {
        uint32_t col = on ? blend(base, 0xFFE0A0, 120) : shade(base, -34);
        fillRect(ren, box, col);
        frameRect(ren, box, on ? 0xFFE0A0 : shade(base, 40));
        // 켜짐/꺼짐은 글자 대신 네모를 그린다 (글꼴에 없는 기호를 쓰면 두부가 나온다)
        {
            int sq = std::max(4, w2sLen(14));
            Rect m{ cx - sq/2, box.y + box.h/2 - sq/2 - w2sLen(4), sq, sq };
            if (on) fillRect(ren, m, 0x201804);
            else    frameRect(ren, m, COL_TEXT);
        }
        if (viewZoom > 0.55f) {
            char b2[24];
            std::snprintf(b2, sizeof(b2), "%d틱", CLOCK_TICKS[std::clamp(c.aux, 0, CLOCK_N-1)]);
            drawTextC(ren, cx, box.y + box.h - w2sLen(15), std::max(7, (int)(10 * viewZoom)),
                      on ? 0x201804 : shade(base, 70), b2);
        }
    } else if (c.type == RAM) {
        fillRect(ren, box, shade(base, -14));
        frameRect(ren, box, shade(base, 45));
        drawTextC(ren, cx, box.y + box.h/2 - ts, ts, COL_TEXT, "램");
        if (viewZoom > 0.5f) {
            char b2[32];
            Val addr = c.in.empty() ? 0 : c.in[0];
            std::snprintf(b2, sizeof(b2), "%d→%d", (int)addr, (int)(c.out.empty()?0:c.out[0]));
            drawTextC(ren, cx, box.y + box.h/2 + w2sLen(4), std::max(7, (int)(11 * viewZoom)),
                      0xE0C0E8, b2);
        }
    } else if (c.type == SCREEN) {
        // n줄 × m칸 점 격자. 입력 하나가 한 줄, 비트 하나가 점 하나.
        fillRect(ren, box, 0x14181E);
        frameRect(ren, box, shade(base, 40));
        int rows = screenRows(c), wd = widthOf(c);
        int pad = std::max(2, w2sLen(5));
        int gw = (box.w - pad * 2) / std::max(1, wd);
        int gh = (box.h - pad * 2 - w2sLen(14)) / std::max(1, rows);
        int d = std::max(1, std::min(gw, gh) - std::max(1, w2sLen(2)));
        for (int r = 0; r < rows; ++r) {
            Val v = (r < (int)c.in.size()) ? c.in[r] : 0;
            for (int k = 0; k < wd; ++k) {
                bool on = (v >> (wd - 1 - k)) & 1;      // 왼쪽이 윗자리
                Rect p2{ box.x + pad + k * gw + (gw - d) / 2,
                         box.y + pad + r * gh + (gh - d) / 2, d, d };
                fillRect(ren, p2, on ? 0x7ADCA0 : 0x232A32);
            }
        }
        if (viewZoom > 0.5f) {
            char b2[24]; std::snprintf(b2, sizeof(b2), "%d×%d", rows, wd);
            drawTextC(ren, cx, box.y + box.h - w2sLen(13), std::max(7, (int)(10 * viewZoom)),
                      0x6E8494, b2);
        }
    } else if (c.type == BUNDLE || c.type == SPLIT) {
        bool fold = isFolded(c);
        fillRect(ren, box, shade(base, fold ? -24 : -14));
        frameRect(ren, box, shade(base, 45));
        if (fold) {
            // 접힘 — 이름을 짧게 쓰고 접혔다는 표시를 그린다
            char b2[24]; std::snprintf(b2, sizeof(b2), "%d", widthOf(c));
            drawTextC(ren, cx, box.y + box.h/2 - ts*3/5, ts, COL_TEXT, b2);
            int m = std::max(2, w2sLen(4));
            for (int k = 0; k < 3; ++k)      // 접힌 가닥을 뜻하는 잔줄
                fillRect(ren, { box.x + box.w/2 - m*3, box.y + box.h - m*3 + k*m,
                                m*6, std::max(1, m/2) }, shade(base, 70));
        } else {
            drawTextC(ren, cx, box.y + box.h/2 - ts, ts, COL_TEXT, compName(c));
            if (viewZoom > 0.5f) {
                char b2[24]; std::snprintf(b2, sizeof(b2), "%d비트", widthOf(c));
                drawTextC(ren, cx, box.y + box.h/2 + w2sLen(4), std::max(7, (int)(11 * viewZoom)),
                          0xA0D0D0, b2);
            }
        }
    } else if (c.type == SWITCH || c.type == LAMP) {
        uint32_t col = on ? blend(base, COL_ON, 150) : shade(base, -30);
        fillRect(ren, box, col);
        frameRect(ren, box, on ? COL_ON : shade(base, 30));
        drawTextC(ren, cx, box.y + box.h/2 - ts*3/5, ts,
                  on ? 0x0E140F : COL_TEXT, on ? "ON" : (c.type == SWITCH ? "OFF" : "·"));
        if (viewZoom > 0.55f)
            drawTextC(ren, cx, box.y + box.h - w2sLen(15), std::max(7, (int)(11 * viewZoom)),
                      on ? 0x0E140F : COL_DIM, compName(c));
    } else {
        fillRect(ren, box, shade(base, on ? 24 : -16));
        frameRect(ren, box, on ? COL_ON : shade(base, 40));
        drawTextC(ren, cx, box.y + box.h/2 - ts*3/5, ts, COL_TEXT, compName(c));
    }

    if (inSel(sel, idx)) {
        frameRect(ren, { box.x-2, box.y-2, box.w+4, box.h+4 }, COL_SEL);
        frameRect(ren, { box.x-3, box.y-3, box.w+6, box.h+6 }, COL_SEL);
    }

    // 출력 쪽에 방향 표시 — 돌렸을 때 어디로 나가는지 한눈에 보이게
    if (viewZoom > 0.4f && nOut(c) > 0) {
        int dx, dy; outDir(c, dx, dy);
        int mx = box.x + box.w/2 + dx * (box.w/2 - w2sLen(7));
        int my = box.y + box.h/2 + dy * (box.h/2 - w2sLen(7));
        fillCircle(ren, mx, my, std::max(1, w2sLen(2)), shade(base, 70));
    }

    int pr = std::max(2, w2sLen(PORT_R));
    for (int p = 0; p < nOut(c); ++p) {
        int px, py; outPort(c, p, px, py);
        bool v = p < (int)c.out.size() && c.out[p] != 0;
        int wd = p < (int)c.outW.size() ? c.outW[p] : 1;
        drawPort(ren, w2sX((float)px), w2sY((float)py), pr, wd, v);
    }
    for (int p = 0; p < nIn(c); ++p) {
        int px, py; inPort(c, p, px, py);
        bool v = p < (int)c.in.size() && c.in[p] != 0;
        int wd = p < (int)c.inW.size() ? c.inW[p] : 1;
        drawPort(ren, w2sX((float)px), w2sY((float)py), pr, wd, v);
    }
}

static int drawnSelWire = -1;      // 지금 골라 둔 선 (그리기용)

static void drawWire(SDL_Renderer* ren, const Wire& w) {
    const Comp& a = world.comps[w.from];
    int pts[24][2]; wireCurve(w, pts, 24);
    bool on = w.fromPort < (int)a.out.size() && a.out[w.fromPort] != 0;
    int wd = (w.fromPort < (int)a.outW.size()) ? a.outW[w.fromPort] : 1;
    uint32_t col = (wd > 1) ? (on ? 0x6AC8E0 : 0x3A4A54)      // 굵은 선은 파랑끼
                            : (on ? COL_ON : COL_OFF);
    int th = std::max(1, w2sLen(wd > 1 ? (on ? 7 : 6) : (on ? 4 : 3)));
    // 골라 둔 선은 굵고 노랗게 — Del 로 지울 대상이 뭔지 보이게
    bool picked = (&w >= world.wires.data() &&
                   (int)(&w - world.wires.data()) == drawnSelWire);
    if (picked) { col = COL_SEL; th += std::max(2, w2sLen(3)); }
    for (int k = 0; k < 23; ++k)
        thickLine(ren, w2sX((float)pts[k][0]),   w2sY((float)pts[k][1]),
                       w2sX((float)pts[k+1][0]), w2sY((float)pts[k+1][1]), th, col);
}

static void drawPanel(SDL_Renderer* ren, int tool, int selCount, int hoverTool, int frame) {
    fillRect(ren, { 0, 0, PANEL_W, WIN_H }, COL_PANEL);
    fillRect(ren, { PANEL_W - 1, 0, 1, WIN_H }, COL_LINE);
    drawText(ren, S(14), S(12), 18, COL_TEXT, "셈틀");

    // 찾기 칸
    {
        Rect fb = findBox();
        fillRect(ren, fb, findOn ? 0x2A3244 : 0x20242E);
        frameRect(ren, fb, findOn ? 0x6C86C8 : 0x2E3440);
        std::string shownTxt = toolFind.empty() && !findOn ? "찾기 (/)" : toolFind;
        if (findOn) shownTxt += ((frame / 30) % 2) ? "_" : " ";
        drawText(ren, fb.x + S(8), fb.y + (fb.h - S(13)) / 2, 13,
                 (toolFind.empty() && !findOn) ? 0x5E6472 : 0xD8DCE6, shownTxt.c_str());
    }

    clampToolScroll();
    std::vector<int> shown = shownTools();
    int nt = (int)shown.size();
    // 목록 자리 밖으로 나간 줄은 안 그린다
    SDL_Rect listClip{ 0, toolTop() - S(4), PANEL_W, toolBot() - toolTop() + S(8) };
    SDL_RenderSetClipRect(ren, &listClip);
    for (int row = 0; row < nt; ++row) {
        int i = shown[row];
        Rect r = toolRect(row);
        if (r.y + r.h < toolTop() - S(8)) continue;
        if (r.y > toolBot() + S(8)) break;
        bool on = (i == tool);
        bool can = toolEnabled(i);
        fillRect(ren, r, on ? 0x2E3550 : (can ? 0x232733 : 0x1C1F27));
        if (on) frameRect(ren, r, 0x6C86C8);
        int ty = r.y + (r.h - S(15)) / 2;              // 줄이 좁아져도 가운데
        if (toolHasColor(i))
            fillRect(ren, { r.x + S(7), r.y + (r.h - S(11)) / 2, S(11), S(11) },
                     can ? toolColor(i) : shade(toolColor(i), -80));
        int cid = toolChip(i);
        drawText(ren, r.x + (toolHasColor(i) ? S(26) : S(9)), ty, 14,
                 on ? 0xFFFFFF : (can ? COL_TEXT : 0x4E5462), toolName(i));
        (void)0;
        if (cid >= 0) {
            Rect d = toolDelBtn(row);
            bool hot = (row == hoverTool);
            bool used = chipUses(cid) > 0 || editingChip(cid);
            fillRect(ren, d, hot ? (used ? 0x4A3038 : 0x5A2A32) : 0x2A2E38);
            uint32_t xc = used ? 0x70707E : (hot ? 0xFFC0C8 : 0xA0A4B0);
            drawTextC(ren, d.x + d.w/2, d.y + (d.h - S(13))/2, 13, xc, "×");

            Rect ed = toolEditBtn(row);
            bool nowEditing = editingChip(cid);
            fillRect(ren, ed, nowEditing ? 0x2C4A6A : 0x2A2E38);
            // 연필 그림(✎)은 글꼴에 없어서 두부처럼 나온다. 글자로 쓴다.
            drawTextC(ren, ed.x + ed.w/2, ed.y + (ed.h - S(11))/2, 11,
                      nowEditing ? 0x9FC0FF : 0xA0A4B0, "고");
        } else if (i == 0) {
            drawText(ren, r.x + r.w - S(18), ty + S(2), 11, COL_DIM, "`");
        } else if (i <= TYPE_N && i <= 9) {          // 0 은 배율 되돌리기라 안 쓴다
            char k[2] = { (char)('0' + i), 0 };
            drawText(ren, r.x + r.w - S(16), ty + S(2), 11, on ? 0x9FC0FF : COL_DIM, k);
        }
    }

    SDL_RenderSetClipRect(ren, nullptr);   // 목록 자르기 끝 — 안 풀면 뒤에 그리는 게 다 잘린다

    // 굴림 막대 — 넘칠 때만 보인다. 잡고 끌 수 있다.
    if (toolScrollMax() > 0) {
        fillRect(ren, scrollTrack(), 0x22262E);
        fillRect(ren, scrollThumb(), 0x5A6070);
    }

    // 찾는 중인데 걸린 게 없으면 알려 준다
    if (!toolFind.empty() && nt <= 1)
        drawText(ren, S(14), toolTop() + S(10), 12, 0x6E7484, "찾는 게 없다");

    Rect bb = btnBundle();
    bool can = selCount > 0;
    fillRect(ren, bb, can ? 0x2C4A3A : 0x232733);
    char b[48]; std::snprintf(b, sizeof(b), can ? "게이트로 묶기 (%d개)" : "게이트로 묶기", selCount);
    drawTextC(ren, bb.x + bb.w/2, bb.y + (bb.h - S(14))/2, 14, can ? 0xB0E0C0 : COL_DIM, b);

    Rect bc = btnClear();
    fillRect(ren, bc, 0x3A2730);
    drawTextC(ren, bc.x + bc.w/2, bc.y + (bc.h - S(14))/2, 14, 0xE0B0B8, "전체 지우기");
}

// ─────────────────────────────────────────────────────────────
// 첫 화면
// ─────────────────────────────────────────────────────────────

static Rect menuBtn(int i) {
    int w = std::min(S(420), WIN_W - S(120)), h = S(78), gap = S(18);
    int top = WIN_H/2 - (h * 2 + gap) / 2 + S(40);   // 제목 아래로 조금 내려서
    return { WIN_W/2 - w/2, top + i * (h + gap), w, h };
}

// ─────────────────────────────────────────────────────────────
// 단계 고르기 화면
//
// 서른 단계를 부로 묶어 늘어놓는다. 통과한 것과 지금 것과 아직 잠긴 것을
// 한눈에 보이게 하고, 통과한 데로는 되돌아갈 수 있게 한다.
// ─────────────────────────────────────────────────────────────

static int pickScroll = 0;               // 목록을 굴린 만큼 (픽셀)

static int pickColW() { return std::min(S(620), WIN_W - S(80)); }
static int pickColX() { return WIN_W/2 - pickColW()/2; }
static int pickTop()  { return S(96); }               // 제목 아래
static int pickBot()  { return WIN_H - S(46); }       // 도움말 위
static int pickPartH(){ return S(52); }
static int pickLesH() { return S(34); }

// 부 머리 하나 + 그 안의 단계들. 위에서부터 쌓은 높이를 돌려준다.
static int pickHeight() {
    return PART_N * pickPartH() + LESSON_N * pickLesH() + PART_N * S(10);
}
static int pickMaxScroll() { return std::max(0, pickHeight() - (pickBot() - pickTop())); }

// 그 단계 줄이 목록 어디쯤에 있나 (맨 위에서부터의 높이)
static int pickRowY(int at) {
    int y = 0;
    for (int p = 0; p < PART_N; ++p) {
        y += pickPartH();
        for (int i = PARTS[p].first; i < partEnd(p); ++i) {
            if (i == at) return y;
            y += pickLesH();
        }
        y += S(10);
    }
    return 0;
}
// 지금 단계가 가운데쯤 오게 굴린다
static void pickScrollTo(int at) {
    int room = pickBot() - pickTop();
    pickScroll = std::clamp(pickRowY(at) - room / 2, 0, pickMaxScroll());
}

// 화면에서 그 자리에 있는 단계 (없으면 -1). 그리기와 누르기가 같은 셈을 쓴다.
static int pickHit(int mx, int my) {
    if (mx < pickColX() || mx > pickColX() + pickColW()) return -1;
    if (my < pickTop() || my > pickBot()) return -1;
    int y = pickTop() - pickScroll;
    for (int p = 0; p < PART_N; ++p) {
        y += pickPartH();
        for (int i = PARTS[p].first; i < partEnd(p); ++i) {
            if (my >= y && my < y + pickLesH()) return i;
            y += pickLesH();
        }
        y += S(10);
    }
    return -1;
}

static void drawPick(SDL_Renderer* ren, int mx, int my) {
    setCol(ren, 0x101219); SDL_RenderClear(ren);
    setCol(ren, 0x171A23);
    for (int x = 0; x < WIN_W; x += S(40)) SDL_RenderDrawLine(ren, x, 0, x, WIN_H);
    for (int y = 0; y < WIN_H; y += S(40)) SDL_RenderDrawLine(ren, 0, y, WIN_W, y);

    drawText(ren, pickColX(), S(34), 30, 0xE8ECF4, "학습");
    char b[80];
    std::snprintf(b, sizeof(b), "%d / %d 단계 통과", lessonDone, LESSON_N);
    drawText(ren, pickColX() + pickColW() - textWidth(14, b), S(46), 14, 0x6E7484, b);

    SDL_Rect clip{ 0, pickTop(), WIN_W, pickBot() - pickTop() };
    SDL_RenderSetClipRect(ren, &clip);

    int hit = pickHit(mx, my);
    int x0 = pickColX(), w = pickColW();
    int y = pickTop() - pickScroll;
    for (int p = 0; p < PART_N; ++p) {
        int lo = PARTS[p].first, hi = partEnd(p);
        int done = std::min(std::max(lessonDone - lo, 0), hi - lo);
        bool locked = (lo > lessonDone);

        std::snprintf(b, sizeof(b), "%d부 · %s", p + 1, PARTS[p].name);
        drawText(ren, x0, y + S(4), 18, locked ? 0x555B69 : 0xC8CEDC, b);
        drawDiff(ren, x0 + w - diffW(), y + S(9), PARTS[p].diff, locked);
        drawText(ren, x0 + w - diffW() - S(8) - textWidth(12, "난이도"), y + S(8), 12,
                 locked ? 0x434956 : 0x5E6472, "난이도");
        drawText(ren, x0, y + S(28), 13, locked ? 0x434956 : 0x767C8C, PARTS[p].what);
        std::snprintf(b, sizeof(b), "%d / %d", done, hi - lo);
        drawText(ren, x0 + w - diffW() - S(14) - textWidth(13, b), y + S(28), 13,
                 done == hi - lo ? 0x6AA88A : 0x5E6472, b);
        y += pickPartH();

        for (int i = lo; i < hi; ++i) {
            bool pass = (i < lessonDone), here = (i == lessonAt), shut = (i > lessonDone);
            Rect r{ x0, y, w, pickLesH() - S(4) };
            if (i == hit && !shut) fillRect(ren, r, 0x252A36);
            else if (here)         fillRect(ren, r, 0x1E2632);
            if (here) fillRect(ren, { r.x, r.y, S(3), r.h }, 0x5A86BA);

            // 상태 표시 — 통과는 채운 네모, 지금은 반쯤, 잠긴 것은 빈 테두리
            Rect m{ x0 + S(14), y + S(9), S(11), S(11) };
            if (pass)      fillRect(ren, m, 0x4A9E6A);
            else if (here) fillRect(ren, m, 0x5A86BA);
            else           frameRect(ren, m, 0x3E4554);

            std::snprintf(b, sizeof(b), "%d", i + 1);
            drawText(ren, x0 + S(38), y + S(7), 14,
                     shut ? 0x474D5A : 0x767C8C, b);
            const char* t = LESSONS[i].title;
            while (*t && *t != ' ') ++t;                 // 제목 앞 번호는 뗀다
            while (*t == ' ') ++t;
            drawText(ren, x0 + S(70), y + S(6), 16,
                     shut ? 0x4E5462 : (here ? 0xFFFFFF : 0xC0C6D4), t);
            if (LESSONS[i].bank && LESSONS[i].name && *LESSONS[i].name) {
                std::snprintf(b, sizeof(b), "%s", LESSONS[i].name);
                drawText(ren, x0 + w - textWidth(13, b) - S(6), y + S(8), 13,
                         shut ? 0x434956 : (pass ? 0x6AA88A : 0x5E6472), b);
            }
            y += pickLesH();
        }
        y += S(10);
    }
    SDL_RenderSetClipRect(ren, nullptr);

    // 굴림 막대
    int room = pickBot() - pickTop(), full = pickHeight();
    if (full > room) {
        int bh = std::max(S(30), room * room / full);
        int by = pickTop() + (room - bh) * pickScroll / std::max(1, pickMaxScroll());
        fillRect(ren, { x0 + pickColW() + S(10), pickTop(), S(4), room }, 0x1D212B);
        fillRect(ren, { x0 + pickColW() + S(10), by, S(4), bh }, 0x3E4554);
    }

    drawTextC(ren, WIN_W/2, WIN_H - S(30), 13, 0x484E5C,
              "누르면 그 단계로 · 통과한 데까지만 갈 수 있다 · Esc 로 첫 화면");
}

static void drawMenu(SDL_Renderer* ren, int hover) {
    setCol(ren, 0x101219); SDL_RenderClear(ren);

    // 배경에 옅은 눈금
    setCol(ren, 0x171A23);
    for (int x = 0; x < WIN_W; x += S(40)) SDL_RenderDrawLine(ren, x, 0, x, WIN_H);
    for (int y = 0; y < WIN_H; y += S(40)) SDL_RenderDrawLine(ren, 0, y, WIN_W, y);

    int titleY = std::max(S(40), menuBtn(0).y - S(190));
    drawTextC(ren, WIN_W/2, titleY, 64, 0xE8ECF4, "셈틀");
    drawTextC(ren, WIN_W/2, titleY + S(86), 16, 0x7A8090, "배선과 게이트만으로 회로를 조합하는 판");

    struct { const char* t; const char* d; uint32_t c; } B[2] = {
        { "학습",     "NAND 하나로 시작해서 계산기까지", 0x35688A },
        { "샌드박스", "빈 판. 마음대로",                 0x2E7D5B },
    };
    for (int i = 0; i < 2; ++i) {
        Rect r = menuBtn(i);
        bool on = (i == hover);
        fillRect(ren, r, on ? shade(B[i].c, -18) : 0x1D212B);
        frameRect(ren, r, on ? shade(B[i].c, 60) : 0x2E3440);
        fillRect(ren, { r.x, r.y, S(5), r.h }, B[i].c);
        drawText(ren, r.x + S(26), r.y + S(15), 27, on ? 0xFFFFFF : 0xD8DCE6, B[i].t);
        drawText(ren, r.x + S(26), r.y + S(50), 14, 0x767C8C, B[i].d);
    }

    char b[64];
    std::snprintf(b, sizeof(b), "학습 진도  %d / %d", lessonDone, LESSON_N);
    drawTextC(ren, WIN_W/2, menuBtn(1).y + menuBtn(1).h + S(26), 14, 0x5E6472, b);
    drawTextC(ren, WIN_W/2, WIN_H - S(36), 13, 0x484E5C,
              "고르면 바로 시작 · 안에서 Esc 로 여기 돌아옴 · [ ] 로 글자 크기");
}

// ─────────────────────────────────────────────────────────────
// 저장·불러오기
//
// 칩 설계도와 판을 통째로 텍스트 한 파일에 쓴다. 바뀌면 알아서 저장한다.
//
// 상자 인스턴스의 속 회로는 저장하지 않는다 — 설계도에서 다시 찍어 내면 되고,
// 그래야 파일이 중첩 깊이만큼 부풀지 않는다. 대신 스위치가 켜졌는지는 저장한다.
// 스위치만 제자리면 나머지는 몇 틱 돌면 저절로 제 값을 찾는다.
// (칩 속 래치가 기억하던 값은 안 남는다. 껐다 켜면 초기 상태부터다.)
// ─────────────────────────────────────────────────────────────

// 모드마다 저장본이 따로다. 문제 풀다 샌드박스에 가도 내 회로가 그대로 있다.
// 저장 형식 번호. 1 = 예전(이름이 '논리'이던 시절 포함), 2 = 폭·클럭·램이 생긴 뒤.
static const int SAVE_VER = 2;

static std::string savePathFor(int sc) {
    const char* over = env2("SEMTLE_SAVE", "LOGIC_SAVE");   // 검사에서 딴 데를 쓰게
    if (over) return sc == SC_LEARN ? std::string(over) + ".learn" : std::string(over);
    const char* home = SDL_getenv("HOME");
    const char* base = (sc == SC_LEARN) ? "learn.txt" : "state.txt";
    if (!home) return std::string("semtle-") + base;
    return std::string(home) + "/.local/share/semtle/" + base;
}
// 목록 화면은 학습에 딸린 것이라 학습 파일에 쓴다. 첫 화면은 샌드박스 쪽으로 친다.
static std::string savePath() {
    int sc = (screen == SC_PICK) ? SC_LEARN : (screen == SC_MENU ? SC_SANDBOX : screen);
    return savePathFor(sc);
}

// 만든 칩은 모드와 상관없이 한 곳에 둔다. 학습에서 딴 걸 샌드박스에서도 쓴다.
static std::string chipsPath() {
    const char* over = env2("SEMTLE_SAVE", "LOGIC_SAVE");
    if (over) return std::string(over) + ".chips";
    const char* home = SDL_getenv("HOME");
    if (!home) return "semtle-chips.txt";
    return std::string(home) + "/.local/share/semtle/chips.txt";
}

// 부품 하나를 알맞은 크기의 빈 상태로 만든다
static Comp blankComp(int type, int chipId) {
    Comp c; c.type = type; c.chipId = chipId;
    if (chipId >= 0) c.sub = std::make_shared<SubSim>(deepCopy(chips[chipId].tmpl));
    resizePorts(c);
    return c;
}

static void writeSub(std::FILE* f, const SubSim& s) {
    std::fprintf(f, "부품 %d\n", (int)s.comps.size());
    for (const auto& c : s.comps)
        // aux(폭·클럭주기) 가 늘었다. 이름은 빈칸이 들어갈 수 있으니 줄 맨 끝.
        std::fprintf(f, "%d %d %d %d %d %d %d %d %s\n", c.type, c.chipId, c.x, c.y, c.rot,
                     (int)c.alive, (int)(c.out.empty() ? 0 : c.out[0]), c.aux, c.label.c_str());
    // 램 속은 따로 (0 이 아닌 자리만)
    for (int i = 0; i < (int)s.comps.size(); ++i) {
        const Comp& c = s.comps[i];
        if (c.type != RAM || c.mem.empty()) continue;
        int used = 0;
        for (Val v : c.mem) if (v) ++used;
        std::fprintf(f, "램속 %d %d", i, used);
        for (int k = 0; k < (int)c.mem.size(); ++k)
            if (c.mem[k]) std::fprintf(f, " %d %d", k, (int)c.mem[k]);
        std::fprintf(f, "\n");
    }
    std::fprintf(f, "램끝\n");
    std::fprintf(f, "선 %d\n", (int)s.wires.size());
    for (const auto& w : s.wires)
        std::fprintf(f, "%d %d %d %d %d\n", w.from, w.fromPort, w.to, w.toPort, (int)w.alive);
    std::fprintf(f, "입력 %d", (int)s.inPorts.size());
    for (int i : s.inPorts) std::fprintf(f, " %d", i);
    std::fprintf(f, "\n출력 %d", (int)s.outPorts.size());
    for (int i : s.outPorts) std::fprintf(f, " %d", i);
    std::fprintf(f, "\n");
}

static int fileVer = SAVE_VER;      // 지금 읽는 중인 파일의 형식 번호

static bool readSub(std::FILE* f, SubSim& s) {
    int n = 0;
    if (std::fscanf(f, " 부품 %d", &n) != 1 || n < 0 || n > 100000) return false;
    s.comps.clear();
    for (int i = 0; i < n; ++i) {
        int type, chipId, x, y, rot, alive, sw, aux = 0;
        if (std::fscanf(f, " %d %d %d %d %d %d %d",
                        &type, &chipId, &x, &y, &rot, &alive, &sw) != 7) return false;
        // aux 는 형식 2부터 있다. 예전 파일에는 아예 없다.
        if (fileVer >= 2 && std::fscanf(f, " %d", &aux) != 1) return false;
        // 표에 없는 번호가 들어오면 파일이 깨진 것이다
        if (chipId >= (int)chips.size()) return false;
        if (chipId < 0 && (type < 0 || type >= TYPE_N)) return false;
        // 줄 나머지가 이름이다 (없으면 빈 칸 — 예전 저장본이 그렇다)
        char rest[160] = "";
        if (!std::fgets(rest, sizeof(rest), f)) rest[0] = 0;
        std::string lab = rest;
        while (!lab.empty() && (lab.back()=='\n' || lab.back()=='\r' || lab.back()==' ')) lab.pop_back();
        size_t b0 = lab.find_first_not_of(' ');
        lab = (b0 == std::string::npos) ? "" : lab.substr(b0);

        Comp c = blankComp(type, chipId);
        c.x = x; c.y = y; c.rot = rot & 3; c.alive = alive != 0;
        c.label = lab;
        if (chipId < 0 && hasWidth(type)) {
            // 아랫자리 = 폭, 0x100 = 접힘, 윗자리 = 화면 줄 수
            c.aux = std::clamp(aux & 0xFF, 1, WIDTH_MAX) | (aux & FOLD_BIT)
                  | (aux & (0x1F << SCR_ROWS_SHIFT));
            resizePorts(c);
        }
        else if (chipId < 0 && type == CLOCK) c.aux = std::clamp(aux, 0, CLOCK_N - 1);
        if (sw && !c.out.empty()) c.out[0] = maskTo((Val)sw, c.outW.empty() ? 1 : c.outW[0]);
        s.comps.push_back(std::move(c));
    }
    // 램 속 (형식 2부터)
    if (fileVer >= 2) { long pos = std::ftell(f);
      char tag[32] = "";
      while (std::fscanf(f, " %31s", tag) == 1 && std::strcmp(tag, "램속") == 0) {
          int idx = 0, used = 0;
          if (std::fscanf(f, " %d %d", &idx, &used) != 2 || used < 0 || used > RAM_WORDS) return false;
          for (int k = 0; k < used; ++k) {
              int at = 0, v = 0;
              if (std::fscanf(f, " %d %d", &at, &v) != 2) return false;
              if (idx >= 0 && idx < (int)s.comps.size() && s.comps[idx].type == RAM) {
                  if (s.comps[idx].mem.size() != (size_t)RAM_WORDS) s.comps[idx].mem.assign(RAM_WORDS, 0);
                  if (at >= 0 && at < RAM_WORDS) s.comps[idx].mem[at] = maskTo((Val)v, RAM_DATAW);
              }
          }
          pos = std::ftell(f);
      }
      if (std::strcmp(tag, "램끝") != 0) std::fseek(f, pos, SEEK_SET);   // 예전 파일
    }

    if (std::fscanf(f, " 선 %d", &n) != 1 || n < 0 || n > 400000) return false;
    s.wires.clear();
    for (int i = 0; i < n; ++i) {
        int from, fp, to, tp, alive;
        if (std::fscanf(f, " %d %d %d %d %d", &from, &fp, &to, &tp, &alive) != 5) return false;
        if (from < 0 || from >= (int)s.comps.size() ||
            to   < 0 || to   >= (int)s.comps.size()) return false;
        s.wires.push_back({ from, fp, to, tp, alive != 0 });
    }
    auto readList = [&](const char* key, std::vector<int>& v) {
        char fmt[32]; std::snprintf(fmt, sizeof(fmt), " %s %%d", key);
        int m = 0;
        if (std::fscanf(f, fmt, &m) != 1 || m < 0 || m > 10000) return false;
        v.clear();
        for (int i = 0; i < m; ++i) {
            int t; if (std::fscanf(f, " %d", &t) != 1) return false;
            if (t < 0 || t >= (int)s.comps.size()) return false;
            v.push_back(t);
        }
        return true;
    };
    if (!readList("입력", s.inPorts)) return false;
    if (!readList("출력", s.outPorts)) return false;
    return true;
}

static bool saveTo(const std::string& path, bool withChips) {
    size_t slash = path.rfind('/');
    if (slash != std::string::npos) {          // 폴더가 없으면 만든다
        std::string dir = path.substr(0, slash);
        std::string cmd = "mkdir -p '" + dir + "'";
        if (std::system(cmd.c_str()) != 0) { /* 있으면 그만 */ }
    }
    std::string tmp = path + ".tmp";
    std::FILE* f = std::fopen(tmp.c_str(), "w");
    if (!f) return false;

    std::fprintf(f, "셈틀 %d\n", SAVE_VER);
    // 판 저장본에는 칩을 안 쓴다 (칩은 chips.txt 에 따로). 내보내기만 같이 담는다.
    std::fprintf(f, "칩 %d\n", withChips ? (int)chips.size() : 0);
    if (withChips) for (const auto& ch : chips) {
        std::fprintf(f, "칩시작 %u %d %s\n", ch.color, (int)ch.alive, ch.name.c_str());
        writeSub(f, ch.tmpl);
        std::fprintf(f, "칩끝\n");
    }
    std::fprintf(f, "판시작\n");
    writeSub(f, world);
    std::fprintf(f, "판끝\n");
    std::fprintf(f, "보기 %.4f %.1f %.1f\n", viewZoom, viewX, viewY);
    std::fprintf(f, "진도 %d %d %d\n", lessonAt, lessonDone, COURSE_VER);
    std::fprintf(f, "글자배율 %.3f\n", uiScale);
    std::fclose(f);
    // 다 쓴 다음에 갈아치운다. 쓰다 죽어도 예전 파일이 안 깨진다.
    return std::rename(tmp.c_str(), path.c_str()) == 0;
}

// 칩만 따로 쓴다
static bool saveChipsFile() {
    std::string path = chipsPath();
    size_t slash = path.rfind('/');
    if (slash != std::string::npos) {
        std::string cmd = "mkdir -p '" + path.substr(0, slash) + "'";
        if (std::system(cmd.c_str()) != 0) { }
    }
    std::string tmp = path + ".tmp";
    std::FILE* f = std::fopen(tmp.c_str(), "w");
    if (!f) return false;
    std::fprintf(f, "셈틀칩 %d\n", SAVE_VER);
    std::fprintf(f, "칩 %d\n", (int)chips.size());
    for (const auto& ch : chips) {
        std::fprintf(f, "칩시작 %u %d %s\n", ch.color, (int)ch.alive, ch.name.c_str());
        writeSub(f, ch.tmpl);
        std::fprintf(f, "칩끝\n");
    }
    std::fclose(f);
    return std::rename(tmp.c_str(), path.c_str()) == 0;
}

static bool saveState() { return saveTo(savePath(), false) && saveChipsFile(); }

// ─────────────────────────────────────────────────────────────
// 되돌리기
//
// 바뀌기 직전 상태를 통째로 찍어 쌓아 둔다. 회로가 작아서 통째로 찍는 게
// 무엇이 어떻게 바뀌었는지 일일이 적는 것보다 훨씬 간단하고 안 틀린다.
// ─────────────────────────────────────────────────────────────

struct Snapshot { std::vector<Chip> chips; SubSim world; };

static const int UNDO_MAX = 60;

static Chip deepCopyChip(const Chip& c) {
    Chip r; r.name = c.name; r.color = c.color; r.alive = c.alive;
    r.tmpl = deepCopy(c.tmpl);
    return r;
}
static Snapshot takeSnap() {
    Snapshot s;
    s.chips.reserve(chips.size());
    for (const auto& c : chips) s.chips.push_back(deepCopyChip(c));
    s.world = deepCopy(world);
    return s;
}
static void putSnap(const Snapshot& s) {
    chips.clear();
    chips.reserve(s.chips.size());
    for (const auto& c : s.chips) chips.push_back(deepCopyChip(c));
    world = deepCopy(s.world);
}

// 예전 저장본은 스위치·전구가 칩 포트였다. 그것들을 핀으로 바꿔 준다.
// 안 그러면 예전에 만든 칩이 포트를 다 잃는다.
static void migratePorts(SubSim& t) {
    for (size_t k = 0; k < t.inPorts.size(); ++k) {
        int i = t.inPorts[k];
        if (i < 0 || i >= (int)t.comps.size()) continue;
        Comp& c = t.comps[i];
        if (c.type == SWITCH) {
            c.type = PIN_IN;
            if (c.label.empty()) { char b[24]; std::snprintf(b, sizeof(b), "입력%d", (int)k+1); c.label = b; }
        }
    }
    for (size_t k = 0; k < t.outPorts.size(); ++k) {
        int i = t.outPorts[k];
        if (i < 0 || i >= (int)t.comps.size()) continue;
        Comp& c = t.comps[i];
        if (c.type == LAMP) {
            c.type = PIN_OUT;
            if (c.label.empty()) { char b[24]; std::snprintf(b, sizeof(b), "출력%d", (int)k+1); c.label = b; }
        }
    }
}

static bool loadFrom(const std::string& path) {
    std::FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return false;

    std::vector<Chip> oldChips = chips;      // 실패하면 되돌린다
    SubSim oldWorld = world;
    world = SubSim{};                        // 판만 비운다 — 칩은 판이 가리키고 있다
    bool ok = false;
    bool hadChips = false;                   // 이 파일이 칩을 담고 있었나

    do {
        int ver = 0, n = 0;
        // 이름을 바꾸기 전 파일은 머리말이 "논리" 다. 둘 다 받아 준다.
        char head[32] = "";
        if (std::fscanf(f, " %31s %d", head, &ver) != 2) break;
        if (std::strcmp(head, "셈틀") != 0 && std::strcmp(head, "논리") != 0) break;
        if (ver < 1 || ver > SAVE_VER) break;
        fileVer = ver;
        if (std::fscanf(f, " 칩 %d", &n) != 1 || n < 0 || n > 5000) break;
        hadChips = (n > 0);
        if (hadChips) chips.clear();         // 파일이 칩을 담고 있으면 그것으로 갈아친다
        bool bad = false;
        for (int i = 0; i < n && !bad; ++i) {
            unsigned col = 0; int alive = 0;
            if (std::fscanf(f, " 칩시작 %u %d ", &col, &alive) != 2) { bad = true; break; }
            char name[128] = "";
            if (!std::fgets(name, sizeof(name), f)) { bad = true; break; }
            size_t len = std::strlen(name);
            while (len && (name[len-1] == '\n' || name[len-1] == '\r')) name[--len] = 0;
            Chip ch; ch.name = name; ch.color = col; ch.alive = alive != 0;
            // 뒤엣칩이 앞엣칩을 쓸 수 있으니 순서대로 넣어 가며 읽는다
            chips.push_back(std::move(ch));
            if (!readSub(f, chips.back().tmpl)) { bad = true; break; }
            migratePorts(chips.back().tmpl);      // 예전 저장본의 스위치·전구 포트를 핀으로
            char end[32] = "";
            if (std::fscanf(f, " %31s", end) != 1 || std::strcmp(end, "칩끝") != 0) bad = true;
        }
        if (bad) break;

        char tag[32] = "";
        if (std::fscanf(f, " %31s", tag) != 1 || std::strcmp(tag, "판시작") != 0) break;
        if (!readSub(f, world)) break;
        if (std::fscanf(f, " %31s", tag) != 1 || std::strcmp(tag, "판끝") != 0) break;

        float z = 1, vx = 0, vy = 0;
        if (std::fscanf(f, " 보기 %f %f %f", &z, &vx, &vy) == 3) {
            viewZoom = std::clamp(z, ZOOM_MIN, ZOOM_MAX); viewX = vx; viewY = vy;
        }
        int la = 0, ld = 0, cv = 0;               // 진도 (없으면 처음부터)
        if (std::fscanf(f, " 진도 %d %d", &la, &ld) == 2) {
            if (std::fscanf(f, " %d", &cv) != 1) cv = 1;   // 번호 없던 시절 = 1
            if (cv == COURSE_VER) {
                lessonAt   = std::clamp(la, 0, LESSON_N - 1);
                lessonDone = std::clamp(ld, 0, LESSON_N);
            } else {
                // 과정이 바뀌었다. 진도는 처음으로, 얻은 칩은 그대로.
                lessonAt = lessonDone = 0;
                world = SubSim{};
            }
        }
        float us = 0;
        if (std::fscanf(f, " 글자배율 %f", &us) == 1 && us > 0.1f) {
            uiScale = std::clamp(us, UI_MIN, UI_MAX);
            applyUiScale();
        }
        ok = true;
    } while (false);

    std::fclose(f);
    if (!ok) { chips = oldChips; world = oldWorld; }
    return ok;
}

// 칩 파일 읽기
static bool loadChipsFile() {
    std::FILE* f = std::fopen(chipsPath().c_str(), "r");
    if (!f) return false;
    std::vector<Chip> got;
    bool ok = false;
    do {
        int ver = 0, n = 0;
        char head[32] = "";
        if (std::fscanf(f, " %31s %d", head, &ver) != 2) break;
        if (std::strcmp(head, "셈틀칩") != 0 || ver < 1 || ver > SAVE_VER) break;
        fileVer = ver;
        if (std::fscanf(f, " 칩 %d", &n) != 1 || n < 0 || n > 5000) break;
        std::vector<Chip> save = chips;
        chips.clear();                       // readSub 이 chips 크기를 본다
        bool bad = false;
        for (int i = 0; i < n && !bad; ++i) {
            unsigned col = 0; int alive = 0;
            if (std::fscanf(f, " 칩시작 %u %d ", &col, &alive) != 2) { bad = true; break; }
            char name[128] = "";
            if (!std::fgets(name, sizeof(name), f)) { bad = true; break; }
            size_t len = std::strlen(name);
            while (len && (name[len-1] == '\n' || name[len-1] == '\r')) name[--len] = 0;
            Chip ch; ch.name = name; ch.color = col; ch.alive = alive != 0;
            chips.push_back(std::move(ch));
            if (!readSub(f, chips.back().tmpl)) { bad = true; break; }
            migratePorts(chips.back().tmpl);
            char end[32] = "";
            if (std::fscanf(f, " %31s", end) != 1 || std::strcmp(end, "칩끝") != 0) bad = true;
        }
        if (bad) { chips = save; break; }
        got = chips;
        ok = true;
    } while (false);
    std::fclose(f);
    if (ok) chips = got;
    return ok;
}

static bool loadState() {
    chips.clear();
    loadChipsFile();                 // 칩은 모드와 상관없이 한 곳에서
    return loadFrom(savePath());
}

// ─────────────────────────────────────────────────────────────
// 파일 고르기 창
//
// SDL2 에는 파일 창이 없다(SDL3 부터 생겼다). 바탕화면에 이미 깔려 있는
// zenity 나 kdialog 를 잠깐 띄워 쓴다. 둘 다 없으면 조용히 포기한다.
// ─────────────────────────────────────────────────────────────

static bool haveCmd(const char* c) {
    std::string t = std::string("command -v ") + c + " >/dev/null 2>&1";
    return std::system(t.c_str()) == 0;
}

static bool fileDialog(bool forSave, std::string& out) {
    std::string cmd;
    if (haveCmd("zenity")) {
        cmd = forSave
            ? "zenity --file-selection --save --confirm-overwrite "
              "--title='회로 내보내기' --filename='회로.셈틀' 2>/dev/null"
            : "zenity --file-selection --title='회로 가져오기' 2>/dev/null";
    } else if (haveCmd("kdialog")) {
        cmd = forSave ? "kdialog --getsavefilename ./회로.셈틀 2>/dev/null"
                      : "kdialog --getopenfilename . 2>/dev/null";
    } else {
        return false;
    }
    std::FILE* p = popen(cmd.c_str(), "r");
    if (!p) return false;
    char line[1024] = "";
    bool got = std::fgets(line, sizeof(line), p) != nullptr;
    pclose(p);
    if (!got) return false;                       // 취소를 눌렀다
    std::string r = line;
    while (!r.empty() && (r.back() == '\n' || r.back() == '\r')) r.pop_back();
    if (r.empty()) return false;
    out = r;
    return true;
}

// ─────────────────────────────────────────────────────────────
// 검사 (--test)
// ─────────────────────────────────────────────────────────────

static void settle(int n = 60) { for (int i = 0; i < n; ++i) tickSub(world); }
static void setSwitch(int i, bool v) { world.comps[i].out.assign(1, v ? 1 : 0); }

static int failed = 0;
static void check(const char* name, bool ok, const char* detail) {
    std::printf("  %s %s  — %s\n", ok ? "통과" : "실패", name, detail);
    if (!ok) failed = 1;
}

static int lastChipId() { return (int)chips.size() - 1; }

// 이름으로 단계 찾기. 표가 바뀌어도 검사가 안 깨지게.
static int lessonByName(const char* n) {
    for (int i = 0; i < LESSON_N; ++i)
        if (LESSONS[i].name && std::strcmp(LESSONS[i].name, n) == 0) return i;
    return -1;
}
// 제목에 든 말로 찾기 (칩이 안 되는 이해 단계용)
static int lessonByTitle(const char* t) {
    for (int i = 0; i < LESSON_N; ++i)
        if (LESSONS[i].title && std::strstr(LESSONS[i].title, t)) return i;
    return -1;
}

// 각 단계의 정답 회로를 판에 짓는다.
// 검사(29번)와 정답 그림 뽑기가 같은 코드를 쓴다 — 따로 두면 어느 한쪽이 낡는다.
// setupLesson(at) 을 먼저 부르고, 앞 단계 칩들이 chips 에 있어야 한다.
static bool buildSolution(int at, char* err, size_t errN) {
    const Lesson& L = LESSONS[at];
    auto chipNamed = [&](const char* n) {
        for (int i = 0; i < (int)chips.size(); ++i)
            if (chips[i].alive && chips[i].name == n) return i;
        return -1;
    };
    bool broke = false;
    char detail[200] = "";
            int A  = findPin(PIN_IN, L.inName[0]);
            int B  = L.nIn > 1 ? findPin(PIN_IN, L.inName[1]) : -1;
            int C  = L.nIn > 2 ? findPin(PIN_IN, L.inName[2]) : -1;
            int O0 = findPin(PIN_OUT, L.outName[0]);
            int O1 = L.nOut > 1 ? findPin(PIN_OUT, L.outName[1]) : -1;
            int O2 = L.nOut > 2 ? findPin(PIN_OUT, L.outName[2]) : -1;

            // 붙박이 게이트 하나로 끝나는 단계는 표를 보고 알아서 고른다
            auto plain = [&](int type) {
                int g = addComp(world, type, 300, 170);
                addWire(world, A, 0, g, 0);
                if (B >= 0) addWire(world, B, 0, g, 1);
                addWire(world, g, 0, O0, 0);
            };
            // 셋을 둘씩 두 번에 나눠 본다
            auto three = [&](int type) {
                int g1 = addComp(world, type, 270, 150), g2 = addComp(world, type, 400, 220);
                addWire(world, A, 0, g1, 0); addWire(world, B, 0, g1, 1);
                addWire(world, g1, 0, g2, 0); addWire(world, C, 0, g2, 1);
                addWire(world, g2, 0, O0, 0);
            };

            std::string t = L.title;
            if      (t.find("이어 보기") != std::string::npos) addWire(world, A, 0, O0, 0);
            else if (t.find("하나가 둘") != std::string::npos) {
                addWire(world, A, 0, O0, 0); addWire(world, A, 0, O1, 0);
            }
            else if (t.find("AND — 둘 다") != std::string::npos) plain(T_AND);
            else if (t.find("OR — 하나만")  != std::string::npos) plain(T_OR);
            else if (t.find("NOT — 뒤집기") != std::string::npos) plain(T_NOT);
            else if (t.find("XOR — 다를")   != std::string::npos) plain(T_XOR);
            else if (t.find("NAND — AND")   != std::string::npos) plain(T_NAND);
            else if (t.find("NOR — OR")     != std::string::npos) plain(T_NOR);
            else if (t.find("셋 다 켜졌")   != std::string::npos) three(T_AND);
            else if (t.find("셋 중 하나라도") != std::string::npos) three(T_OR);
            else if (t.find("다수결") != std::string::npos) {
                // (A와B) 또는 (B와C) 또는 (A와C)
                int ab = addComp(world, T_AND, 250, 110), bc = addComp(world, T_AND, 250, 230);
                int ac = addComp(world, T_AND, 250, 350);
                int o1 = addComp(world, T_OR, 390, 170), o2 = addComp(world, T_OR, 500, 260);
                addWire(world, A, 0, ab, 0); addWire(world, B, 0, ab, 1);
                addWire(world, B, 0, bc, 0); addWire(world, C, 0, bc, 1);
                addWire(world, A, 0, ac, 0); addWire(world, C, 0, ac, 1);
                addWire(world, ab, 0, o1, 0); addWire(world, bc, 0, o1, 1);
                addWire(world, o1, 0, o2, 0); addWire(world, ac, 0, o2, 1);
                addWire(world, o2, 0, O0, 0);
            }
            else if (t.find("NAND 로 NOT") != std::string::npos) {   // NAND(A, A)
                int g = addComp(world, T_NAND, 300, 170);
                addWire(world, A, 0, g, 0); addWire(world, A, 0, g, 1);
                addWire(world, g, 0, O0, 0);
            }
            else if (t.find("NAND 로 AND") != std::string::npos) {   // NOT(NAND(A,B))
                int ni = chipNamed("NOT짜기");
                if (ni < 0) { std::snprintf(detail, sizeof(detail), "%d단계: NOT짜기 칩이 없다", at+1); broke = true; }
                int g = addComp(world, T_NAND, 250, 180), n = addChip(world, ni, 380, 180);
                addWire(world, A, 0, g, 0); addWire(world, B, 0, g, 1);
                addWire(world, g, 0, n, 0); addWire(world, n, 0, O0, 0);
            }
            else if (t.find("NAND 로 OR") != std::string::npos) {    // NAND(NOT A, NOT B)
                int ni = chipNamed("NOT짜기");
                if (ni < 0) { std::snprintf(detail, sizeof(detail), "%d단계: NOT짜기 칩이 없다", at+1); broke = true; }
                int na = addChip(world, ni, 230, 110), nb = addChip(world, ni, 230, 250);
                int g  = addComp(world, T_NAND, 380, 180);
                addWire(world, A, 0, na, 0); addWire(world, B, 0, nb, 0);
                addWire(world, na, 0, g, 0); addWire(world, nb, 0, g, 1);
                addWire(world, g, 0, O0, 0);
            }
            else if (t.find("NAND 로 XOR") != std::string::npos) {   // AND(OR(A,B), NAND(A,B))
                int oi = chipNamed("OR짜기"), ai = chipNamed("AND짜기");
                if (oi < 0 || ai < 0) { std::snprintf(detail, sizeof(detail), "%d단계: 앞 칩이 없다", at+1); broke = true; }
                int o = addChip(world, oi, 230, 100);
                int g = addComp(world, T_NAND, 240, 260);
                int a2 = addChip(world, ai, 390, 180);
                addWire(world, A, 0, o, 0); addWire(world, B, 0, o, 1);
                addWire(world, A, 0, g, 0); addWire(world, B, 0, g, 1);
                addWire(world, o, 0, a2, 0); addWire(world, g, 0, a2, 1);
                addWire(world, a2, 0, O0, 0);
            }
            else if (t.find("한 자리 덧셈") != std::string::npos) {  // XOR + AND
                int x = addComp(world, T_XOR, 300, 130), n = addComp(world, T_AND, 300, 260);
                addWire(world, A, 0, x, 0); addWire(world, B, 0, x, 1);
                addWire(world, A, 0, n, 0); addWire(world, B, 0, n, 1);
                addWire(world, x, 0, O0, 0); addWire(world, n, 0, O1, 0);
            }
            else if (t.find("기억하기") != std::string::npos) {       // SR 래치
                // Q = NOR(리셋, Q아님),  Q아님 = NOR(셋, Q)
                int n1 = addComp(world, T_NOR, 300, 120);            // Q 쪽
                int n2 = addComp(world, T_NOR, 300, 300);            // Q아님 쪽
                addWire(world, B, 0, n1, 0);                          // 리셋
                addWire(world, n2, 0, n1, 1);
                addWire(world, A, 0, n2, 0);                          // 셋
                addWire(world, n1, 0, n2, 1);
                addWire(world, n1, 0, O0, 0);
            }
            else if (t.find("잡아 두기") != std::string::npos) {       // D 래치
                int li = chipNamed("SR래치");
                if (li < 0) { std::snprintf(detail, sizeof(detail), "%d단계: SR래치 칩이 없다", at+1); broke = true; }
                int nt2 = addComp(world, T_NOT, 240, 300);
                int as  = addComp(world, T_AND, 360, 130);            // 셋  = 값 & 받아라
                int ar  = addComp(world, T_AND, 360, 300);            // 리셋 = ~값 & 받아라
                int lat = addChip(world, li, 500, 200);
                addWire(world, A, 0, as, 0); addWire(world, B, 0, as, 1);
                addWire(world, A, 0, nt2, 0);
                addWire(world, nt2, 0, ar, 0); addWire(world, B, 0, ar, 1);
                addWire(world, as, 0, lat, 0); addWire(world, ar, 0, lat, 1);
                addWire(world, lat, 0, O0, 0);
            }
            else if (t.find("순간만 잡기") != std::string::npos) {     // 엣지 플립플롭
                int di = chipNamed("D래치");
                if (di < 0) { std::snprintf(detail, sizeof(detail), "%d단계: D래치 칩이 없다", at+1); broke = true; }
                int nc = addComp(world, T_NOT, 240, 320);
                int m  = addChip(world, di, 380, 140);      // 앞엣것 (클럭 꺼졌을 때 받음)
                int sl = addChip(world, di, 560, 220);      // 뒤엣것 (켜졌을 때 받음)
                addWire(world, B, 0, nc, 0);
                addWire(world, A, 0, m, 0); addWire(world, nc, 0, m, 1);
                addWire(world, m, 0, sl, 0); addWire(world, B, 0, sl, 1);
                addWire(world, sl, 0, O0, 0);
            }
            else if (t.find("여덟 칸") != std::string::npos) {         // 8비트 레지스터
                int di = chipNamed("플립플롭");
                if (di < 0) { std::snprintf(detail, sizeof(detail), "%d단계: 플립플롭 칩이 없다", at+1); broke = true; }
                int sp = addComp(world, SPLIT, 220, 200);
                int bu = addComp(world, BUNDLE, 700, 200);
                addWire(world, A, 0, sp, 0);                          // 값(8비트)
                for (int k = 0; k < 8; ++k) {
                    int d = addChip(world, di, 420, 40 + k * 70);
                    addWire(world, sp, k, d, 0);                      // 그 비트
                    addWire(world, B, 0, d, 1);                       // 받아라 (여덟 군데로)
                    addWire(world, d, 0, bu, k);
                }
                addWire(world, bu, 0, O0, 0);
            }
            else if (t.find("여덟 자리 덧셈") != std::string::npos) {  // 8비트 덧셈기
                int fi = chipNamed("전가산기");
                if (fi < 0) { std::snprintf(detail, sizeof(detail), "%d단계: 전가산기 칩이 없다", at+1); broke = true; }
                int spA = addComp(world, SPLIT, 200, 120);
                int spB = addComp(world, SPLIT, 200, 360);
                int bu  = addComp(world, BUNDLE, 800, 200);
                addWire(world, A, 0, spA, 0);
                addWire(world, B, 0, spB, 0);
                int carry = -1;                       // 앞자리에서 넘어온 올림
                for (int k = 0; k < 8; ++k) {
                    int fa = addChip(world, fi, 480, 20 + k * 90);
                    addWire(world, spA, k, fa, 0);
                    addWire(world, spB, k, fa, 1);
                    if (carry >= 0) addWire(world, carry, 1, fa, 2);   // 앞의 올림 → 올림입력
                    addWire(world, fa, 0, bu, k);                      // 합
                    carry = fa;
                }
                addWire(world, bu, 0, O0, 0);
                if (carry >= 0) addWire(world, carry, 1, O1, 0);       // 맨 위 올림 = 넘침
            }
            else if (t.find("세기") != std::string::npos) {         // 세는 상자
                int ri = chipNamed("레지스터"), ai = chipNamed("덧셈기8");
                if (ri < 0 || ai < 0) { std::snprintf(detail, sizeof(detail), "%d단계: 앞 칩이 없다", at+1); broke = true; }
                int reg = addChip(world, ri, 620, 200);
                int add = addChip(world, ai, 380, 320);
                int one = addComp(world, BUNDLE, 200, 420);            // 1 을 만든다
                int sw1 = addComp(world, SWITCH, 60, 420);
                setSwitch(sw1, 1);
                addWire(world, sw1, 0, one, 0);                        // 0번 자리만 켜면 1
                // 되돌리기면 0 이 들어가게 — 여덟 비트를 각각 AND 로 막는다
                int nr = addComp(world, T_NOT, 200, 560);
                addWire(world, B, 0, nr, 0);                           // 되돌리기 아님
                int spS = addComp(world, SPLIT, 500, 520);
                int buG = addComp(world, BUNDLE, 700, 520);
                addWire(world, add, 0, spS, 0);                        // 더한 값을 풀고
                for (int k = 0; k < 8; ++k) {
                    int g = addComp(world, T_AND, 600, 500 + k * 60);
                    addWire(world, spS, k, g, 0);
                    addWire(world, nr, 0, g, 1);                       // 되돌리기면 0
                    addWire(world, g, 0, buG, k);
                }
                addWire(world, buG, 0, reg, 0);                        // 레지스터로
                addWire(world, A, 0, reg, 1);                          // 클럭 = 받아라
                addWire(world, reg, 0, add, 0);                        // 되먹임
                addWire(world, one, 0, add, 1);
                addWire(world, reg, 0, O0, 0);
            }
            else if (t.find("더하고 빼기") != std::string::npos) {     // 계산기
                int ai = chipNamed("덧셈기8");
                if (ai < 0) { std::snprintf(detail, sizeof(detail), "%d단계: 덧셈기8 칩이 없다", at+1); broke = true; }
                // B 를 빼기와 XOR 해서 뒤집고, A 쪽에 빼기(=1)를 더해 2의 보수를 맞춘다
                int spB = addComp(world, SPLIT, 220, 300);
                int buB = addComp(world, BUNDLE, 460, 300);
                addWire(world, B, 0, spB, 0);
                for (int k = 0; k < 8; ++k) {
                    int x = addComp(world, T_XOR, 340, 220 + k * 60);
                    addWire(world, spB, k, x, 0);
                    addWire(world, C, 0, x, 1);            // 빼기
                    addWire(world, x, 0, buB, k);
                }
                int buOne = addComp(world, BUNDLE, 300, 60);   // 빼기면 1
                addWire(world, C, 0, buOne, 0);
                int add1 = addChip(world, ai, 520, 60);        // A + (빼기?1:0)
                addWire(world, A, 0, add1, 0);
                addWire(world, buOne, 0, add1, 1);
                int add2 = addChip(world, ai, 720, 200);       // 그 결과 + 뒤집은 B
                addWire(world, add1, 0, add2, 0);
                addWire(world, buB, 0, add2, 1);
                addWire(world, add2, 0, O0, 0);
            }
            else if (t.find("명령 읽기") != std::string::npos) {       // 해독기
                int sp = addComp(world, SPLIT, 240, 200);
                world.comps[sp].aux = 2; resizePorts(world.comps[sp]);
                addWire(world, A, 0, sp, 0);
                int n0 = addComp(world, T_NOT, 380, 120);      // 아랫자리 아님
                int n1 = addComp(world, T_NOT, 380, 300);      // 윗자리 아님
                addWire(world, sp, 0, n0, 0);
                addWire(world, sp, 1, n1, 0);
                int g0 = addComp(world, T_AND, 540, 100);      // 멈춤
                addWire(world, n0, 0, g0, 0); addWire(world, n1, 0, g0, 1);
                int g1 = addComp(world, T_AND, 540, 220);      // 불러오기
                addWire(world, sp, 0, g1, 0); addWire(world, n1, 0, g1, 1);
                int g2 = addComp(world, T_AND, 540, 340);      // 더하기
                addWire(world, n0, 0, g2, 0); addWire(world, sp, 1, g2, 1);
                addWire(world, g0, 0, O0, 0);
                addWire(world, g1, 0, O1, 0);
                addWire(world, g2, 0, O2, 0);
            }
            else if (t.find("쌓아 두는 곳") != std::string::npos) {    // 누산기
                int ri = chipNamed("레지스터"), ci = chipNamed("계산기");
                if (ri < 0 || ci < 0) { std::snprintf(detail, sizeof(detail), "%d단계: 앞 칩이 없다", at+1); broke = true; }
                int reg = addChip(world, ri, 760, 200);
                int alu = addChip(world, ci, 560, 300);
                int sp  = addComp(world, SPLIT, 220, 420);
                int bu  = addComp(world, BUNDLE, 440, 420);
                addWire(world, reg, 0, sp, 0);                 // 누산기값을 풀어서
                for (int k = 0; k < 8; ++k) {
                    int g = addComp(world, T_AND, 330, 380 + k * 55);
                    addWire(world, sp, k, g, 0);
                    addWire(world, C, 0, g, 1);                // 더할까
                    addWire(world, g, 0, bu, k);
                }
                addWire(world, bu, 0, alu, 0);                 // 계산기 A
                addWire(world, B, 0, alu, 1);                  // 계산기 B = 값
                addWire(world, alu, 0, reg, 0);
                addWire(world, A, 0, reg, 1);                  // 클럭
                addWire(world, reg, 0, O0, 0);
            }
            else if (t.find("스스로 도는 것") != std::string::npos) {   // 컴퓨터
                int ci = chipNamed("세는상자"), di = chipNamed("해독기"), ai = chipNamed("누산기");
                if (ci < 0 || di < 0 || ai < 0) { std::snprintf(detail, sizeof(detail), "%d단계: 앞 칩이 없다", at+1); broke = true; }
                int pc  = addChip(world, ci, 180, 200);
                int ram = addComp(world, RAM, 380, 200);
                int sp  = addComp(world, SPLIT, 560, 200);     // 명령 바이트를 여덟 가닥으로
                int dec = addChip(world, di, 760, 120);
                int acc = addChip(world, ai, 900, 300);

                addWire(world, A, 0, pc, 0);                   // 클럭
                addWire(world, B, 0, pc, 1);                   // 되돌리기
                addWire(world, pc, 0, ram, 0);                 // 세는 값 → 주소
                addWire(world, ram, 0, sp, 0);                 // 명령 바이트

                int opBu = addComp(world, BUNDLE, 660, 60);    // 윗 2비트 → 해독기
                opBu = opBu;
                world.comps[opBu].aux = 2; resizePorts(world.comps[opBu]);
                addWire(world, sp, 6, opBu, 0);
                addWire(world, sp, 7, opBu, 1);
                addWire(world, opBu, 0, dec, 0);

                int valBu = addComp(world, BUNDLE, 660, 420);  // 아랫 6비트 → 누산기 값
                for (int k = 0; k < 6; ++k) addWire(world, sp, k, valBu, k);

                // 넣기나 더하기일 때만 누산기가 받는다
                int orLd = addComp(world, T_OR, 860, 180);
                addWire(world, dec, 1, orLd, 0);               // 불러오기
                addWire(world, dec, 2, orLd, 1);               // 더하기
                // 클럭이 올라갈 때만 실제로 받게 클럭과 AND
                int gate = addComp(world, T_AND, 880, 240);
                addWire(world, orLd, 0, gate, 0);
                addWire(world, A, 0, gate, 1);

                addWire(world, gate, 0, acc, 0);               // 누산기 클럭
                addWire(world, valBu, 0, acc, 1);              // 값
                addWire(world, dec, 2, acc, 2);                // 더할까
                addWire(world, acc, 0, O0, 0);
            }
            else if (t.find("눌러서 넣기") != std::string::npos) {     // 스위치묶음
                int sb = addComp(world, SWBANK, 300, 150);
                world.comps[sb].out.assign(1, 181);
                addWire(world, sb, 0, O0, 0);
            }
            else if (t.find("화면에 찍기") != std::string::npos) {     // 화면
                int sc2 = addComp(world, SCREEN, 400, 120);
                world.comps[sc2].aux = 8 | (8 << SCR_ROWS_SHIFT);
                resizePorts(world.comps[sc2]);
                addWire(world, A, 0, sc2, 0);        // 윗줄 → 0번
                addWire(world, B, 0, sc2, 3);        // 가운뎃줄 → 3번
                addWire(world, A, 0, O0, 0);         // 확인
            }
            else if (t.find("램으로 그림") != std::string::npos) {     // 글자찍개
                int ram = addComp(world, RAM, 400, 150);
                addWire(world, A, 0, ram, 0);        // 줄번호 → 주소
                addWire(world, ram, 0, O0, 0);       // 나온 값 → 그림
            }
            else {                                                   // 전가산기
                int hi = chipNamed("반가산기");
                if (hi < 0) { std::snprintf(detail, sizeof(detail), "%d단계: 반가산기 칩이 없다", at+1); broke = true; }
                int h1 = addChip(world, hi, 230, 100), h2 = addChip(world, hi, 370, 190);
                int orG = addComp(world, T_OR, 510, 300);
                addWire(world, A, 0, h1, 0); addWire(world, B, 0, h1, 1);
                addWire(world, h1, 0, h2, 0); addWire(world, C, 0, h2, 1);
                addWire(world, h2, 0, O0, 0);
                addWire(world, h1, 1, orG, 0); addWire(world, h2, 1, orG, 1);
                addWire(world, orG, 0, O1, 0);
            }
    if (broke) { std::snprintf(err, errN, "%s", detail); return false; }
    return true;
}

// 통과했던 단계의 칩이 없어졌으면 다시 만들어 준다.
// 학습에서 상품으로 받은 칩을 지우고 저장하면 뒷 단계가 영영 안 풀린다.
// 판은 건드리지 않는다.
static int repairLessonChips() {
    SubSim keepWorld = deepCopy(world);
    int keepAt = lessonAt;
    int fixed = 0;
    for (int i = 0; i < LESSON_N && i < lessonDone; ++i) {
        const Lesson& L = LESSONS[i];
        if (!L.bank || !L.name || !*L.name) continue;
        bool have = false;
        for (const auto& c : chips) if (c.alive && c.name == L.name) have = true;
        if (have) continue;
        lessonAt = i;
        setupLesson(i);
        char err[200] = "";
        if (buildSolution(i, err, sizeof(err))) { bankLesson(L); ++fixed; }
        else std::fprintf(stderr, "%d단계 칩을 못 되살림: %s\n", i + 1, err);
    }
    world = std::move(keepWorld);
    lessonAt = keepAt;
    return fixed;
}

static int runTests() {
    char buf[256];
    // 검사가 진짜 저장 파일을 건드리면 안 된다. 따로 지정 안 했으면 임시 파일로.
    if (!env2("SEMTLE_SAVE", "LOGIC_SAVE")) SDL_setenv("SEMTLE_SAVE", "/tmp/semtle-test-state.txt", 1);
    // 글꼴이 없으면 textWidth 가 0 을 돌려줘서 자리 검사가 헛돈다. 화면 없이도 읽어 둔다.
    if (!fontOK) loadFont();
    applyUiScale();                       // 판 너비·최소 창 크기 잡기
    WIN_W = WIN_W0; WIN_H = WIN_H0;

    // 1. 게이트 진리표
    {
        struct { int t; const char* name; bool want[4]; } gs[] = {
            { T_AND,  "AND",  { false, false, false, true  } },
            { T_OR,   "OR",   { false, true,  true,  true  } },
            { T_XOR,  "XOR",  { false, true,  true,  false } },
            { T_NAND, "NAND", { true,  true,  true,  false } },
            { T_NOR,  "NOR",  { true,  false, false, false } },
        };
        bool allOK = true; char detail[128] = "";
        for (auto& g : gs) for (int tc = 0; tc < 4; ++tc) {
            world = SubSim{};
            int sa = addComp(SWITCH), sb = addComp(SWITCH), gate = addComp(g.t), lamp = addComp(LAMP);
            setSwitch(sa, tc & 1); setSwitch(sb, (tc >> 1) & 1);
            addWire(sa, 0, gate, 0); addWire(sb, 0, gate, 1); addWire(gate, 0, lamp, 0);
            settle();
            if (lit(world.comps[lamp]) != g.want[tc]) {
                allOK = false;
                std::snprintf(detail, sizeof(detail), "%s (%d,%d) → %d (원하는 값 %d)",
                              g.name, tc & 1, (tc >> 1) & 1, (int)lit(world.comps[lamp]), (int)g.want[tc]);
            }
        }
        check("게이트가 진리표대로 돈다", allOK,
              allOK ? "AND · OR · XOR · NAND · NOR 전부 맞음" : detail);
    }

    // 2. NOT
    {
        world = SubSim{};
        int s = addComp(SWITCH), nt = addComp(T_NOT), l = addComp(LAMP);
        addWire(s, 0, nt, 0); addWire(nt, 0, l, 0);
        setSwitch(s, false); settle(); bool off = lit(world.comps[l]);
        setSwitch(s, true);  settle(); bool on  = lit(world.comps[l]);
        std::snprintf(buf, sizeof(buf), "입력 0일 때 %d, 1일 때 %d", (int)off, (int)on);
        check("NOT 이 뒤집는다", off && !on, buf);
    }

    // 3. 반가산기
    {
        bool allOK = true; char detail[128] = "";
        for (int tc = 0; tc < 4; ++tc) {
            world = SubSim{};
            int a = addComp(SWITCH), b = addComp(SWITCH);
            int sumG = addComp(T_XOR), carG = addComp(T_AND);
            int sumL = addComp(LAMP),  carL = addComp(LAMP);
            setSwitch(a, tc & 1); setSwitch(b, (tc >> 1) & 1);
            addWire(a, 0, sumG, 0); addWire(b, 0, sumG, 1);
            addWire(a, 0, carG, 0); addWire(b, 0, carG, 1);
            addWire(sumG, 0, sumL, 0); addWire(carG, 0, carL, 0);
            settle();
            int A = tc & 1, B = (tc >> 1) & 1;
            bool wantSum = (A + B) & 1, wantCar = (A + B) >> 1;
            if (lit(world.comps[sumL]) != wantSum || lit(world.comps[carL]) != wantCar) {
                allOK = false;
                std::snprintf(detail, sizeof(detail), "%d+%d → 합 %d 올림 %d",
                              A, B, (int)lit(world.comps[sumL]), (int)lit(world.comps[carL]));
            }
        }
        check("반가산기가 더한다", allOK, allOK ? "네 경우 다 맞음" : detail);
    }

    // 4. NOR 래치가 기억한다
    {
        world = SubSim{};
        int R = addComp(SWITCH), S = addComp(SWITCH);
        int q = addComp(T_NOR), qn = addComp(T_NOR), Lq = addComp(LAMP);
        addWire(R, 0, q, 0); addWire(qn, 0, q, 1);
        addWire(S, 0, qn, 0); addWire(q, 0, qn, 1);
        addWire(q, 0, Lq, 0);
        setSwitch(S, true); settle(); setSwitch(S, false); settle();
        bool afterSet = lit(world.comps[Lq]);
        setSwitch(R, true); settle(); setSwitch(R, false); settle();
        bool afterReset = lit(world.comps[Lq]);
        std::snprintf(buf, sizeof(buf), "셋 뒤 %d, 리셋 뒤 %d", (int)afterSet, (int)afterReset);
        check("래치가 기억한다", afterSet && !afterReset, buf);
    }

    // 5. 커스텀 게이트: 반가산기를 상자로 묶어 쓴다
    {
        world = SubSim{};
        // 판에 반가산기 하나 짜고 묶는다. 핀이 칩의 포트가 된다.
        int a = addComp(PIN_IN, 100, 100), b = addComp(PIN_IN, 100, 200);
        int sumG = addComp(T_XOR, 250, 120), carG = addComp(T_AND, 250, 220);
        int sumL = addComp(PIN_OUT, 400, 120), carL = addComp(PIN_OUT, 400, 220);
        addWire(a, 0, sumG, 0); addWire(b, 0, sumG, 1);
        addWire(a, 0, carG, 0); addWire(b, 0, carG, 1);
        addWire(sumG, 0, sumL, 0); addWire(carG, 0, carL, 0);
        createChip({ a, b, sumG, carG, sumL, carL }, "반가산");
        // 이제 판엔 상자 하나 + 그 상자에 새 스위치·전구를 붙여 검사
        int inst = -1;
        for (int i = 0; i < (int)world.comps.size(); ++i)
            if (world.comps[i].alive && world.comps[i].chipId >= 0) inst = i;
        bool allOK = (inst >= 0);
        char detail[128] = "";
        if (inst >= 0) {
            int sw0 = addComp(SWITCH, 600, 100), sw1 = addComp(SWITCH, 600, 200);
            int lo0 = addComp(LAMP, 800, 100), lo1 = addComp(LAMP, 800, 200);
            addWire(sw0, 0, inst, 0); addWire(sw1, 0, inst, 1);
            addWire(inst, 0, lo0, 0); addWire(inst, 1, lo1, 0);
            for (int tc = 0; tc < 4 && allOK; ++tc) {
                setSwitch(sw0, tc & 1); setSwitch(sw1, (tc >> 1) & 1);
                settle();
                int A = tc & 1, B = (tc >> 1) & 1;
                bool wantSum = (A + B) & 1, wantCar = (A + B) >> 1;
                if (lit(world.comps[lo0]) != wantSum || lit(world.comps[lo1]) != wantCar) {
                    allOK = false;
                    std::snprintf(detail, sizeof(detail), "%d+%d → 합 %d 올림 %d",
                                  A, B, (int)lit(world.comps[lo0]), (int)lit(world.comps[lo1]));
                }
            }
        } else std::snprintf(detail, sizeof(detail), "상자가 안 생김");
        check("커스텀 게이트가 판대로 돈다", allOK,
              allOK ? "묶은 반가산기가 0+0 · 0+1 · 1+0 · 1+1 다 맞음" : detail);
    }

    // 6. 상자 안에 상자 (중첩) — 반가산 두 개로 3입력 다수결 흉내는 복잡하니
    //    같은 반가산 상자를 두 인스턴스 놓고 서로 값이 안 섞이는지 본다
    {
        // world 는 5번에서 반가산 chip 이 만들어진 상태를 잇지 않고 새로 시작하되
        // chip 정의(chips[])는 그대로 재사용
        int chipId = lastChipId();
        world = SubSim{};
        int inA = addChip(world, chipId, 100, 100);
        int inB = addChip(world, chipId, 100, 300);
        int s0 = addComp(SWITCH), s1 = addComp(SWITCH), s2 = addComp(SWITCH), s3 = addComp(SWITCH);
        int la = addComp(LAMP), lb = addComp(LAMP);
        addWire(s0, 0, inA, 0); addWire(s1, 0, inA, 1); addWire(inA, 0, la, 0);
        addWire(s2, 0, inB, 0); addWire(s3, 0, inB, 1); addWire(inB, 0, lb, 0);
        setSwitch(s0, 1); setSwitch(s1, 0);    // A: 1+0 → 합 1
        setSwitch(s2, 1); setSwitch(s3, 1);    // B: 1+1 → 합 0
        settle();
        bool ok = lit(world.comps[la]) == true && lit(world.comps[lb]) == false;
        std::snprintf(buf, sizeof(buf), "상자A 합 %d, 상자B 합 %d (안 섞임)",
                      (int)lit(world.comps[la]), (int)lit(world.comps[lb]));
        check("상자 여럿이 서로 안 섞인다", ok, buf);
    }

    // 7. 선을 끊으면 신호가 안 간다
    {
        world = SubSim{};
        int s = addComp(SWITCH), l = addComp(LAMP);
        addWire(s, 0, l, 0);
        setSwitch(s, true); settle(); bool before = lit(world.comps[l]);
        world.wires[0].alive = false; settle();
        std::snprintf(buf, sizeof(buf), "끊기 전 %d, 끊은 뒤 %d", (int)before, (int)lit(world.comps[l]));
        check("선을 끊으면 안 간다", before && !lit(world.comps[l]), buf);
    }

    // 8. 쓰는 중인 커스텀 게이트는 못 지운다
    {
        world = SubSim{}; chips.clear();
        int a = addComp(PIN_IN, 100, 100), b = addComp(PIN_IN, 100, 200);
        int g = addComp(T_XOR, 250, 120), l = addComp(PIN_OUT, 400, 120);
        addWire(a, 0, g, 0); addWire(b, 0, g, 1); addWire(g, 0, l, 0);
        createChip({ a, b, g, l }, "반가산");
        int id = 0;
        int usedNow = chipUses(id);                 // 판에 인스턴스 하나 놓여 있다
        bool blocked = !deleteChip(id);
        // 판을 비우면 쓰는 데가 없어져 지워진다
        world = SubSim{};
        bool freeNow = (chipUses(id) == 0);
        bool gone = deleteChip(id);
        bool hidden = true;
        for (int c : liveChips()) if (c == id) hidden = false;
        std::snprintf(buf, sizeof(buf), "쓰는 중 %d군데 → 막힘 %d, 비운 뒤 → 지워짐 %d, 목록에서 사라짐 %d",
                      usedNow, (int)blocked, (int)gone, (int)hidden);
        check("쓰는 중인 게이트는 못 지운다", usedNow > 0 && blocked && freeNow && gone && hidden, buf);
    }

    // 9. 다른 게이트 속에 든 것도 쓰는 중으로 센다
    //    반가산을 묶고, 그걸로 전가산을 묶으면, 반가산은 전가산 설계도 안에 산다.
    {
        world = SubSim{}; chips.clear();
        int a = addComp(PIN_IN, 100, 100), b = addComp(PIN_IN, 100, 200);
        int g = addComp(T_XOR, 250, 120), l = addComp(PIN_OUT, 400, 120);
        addWire(a, 0, g, 0); addWire(b, 0, g, 1); addWire(g, 0, l, 0);
        createChip({ a, b, g, l }, "반가산");          // chips[0]
        // 판에 놓인 반가산 상자에 핀을 붙여 통째로 다시 묶는다
        std::vector<int> all;
        for (int i = 0; i < (int)world.comps.size(); ++i) if (world.comps[i].alive) all.push_back(i);
        int s0 = addComp(PIN_IN, 600, 100), l0 = addComp(PIN_OUT, 800, 100);
        all.push_back(s0); all.push_back(l0);
        createChip(all, "전가산");                     // chips[1] 속에 반가산이 들어감
        world = SubSim{};                              // 판은 비운다
        int inner = chipUses(0);                       // 전가산 설계도 속에 살아 있다
        bool blocked = !deleteChip(0);
        bool outerGone = deleteChip(1);                // 바깥것은 아무도 안 쓰니 지워짐
        bool innerNow = deleteChip(0);                 // 그러면 속엣것도 자유
        std::snprintf(buf, sizeof(buf), "속에 %d개 → 막힘 %d, 바깥 지운 뒤 속도 지워짐 %d",
                      inner, (int)blocked, (int)(outerGone && innerNow));
        check("게이트 속에 든 것도 지켜진다", inner > 0 && blocked && outerGone && innerNow, buf);
    }

    // 10. 지워도 남은 게이트들의 번호가 안 밀린다
    //     chipId 가 chips[] 의 자리라, 진짜로 빼면 이미 놓인 상자가 엉뚱한 걸 가리킨다.
    {
        world = SubSim{}; chips.clear();
        auto makeChip = [&](const char* name) {
            int s = addComp(PIN_IN, 100, 100), lp = addComp(PIN_OUT, 300, 100);
            addWire(s, 0, lp, 0);
            createChip({ s, lp }, name);
        };
        makeChip("가"); makeChip("나"); makeChip("다");   // chips 0,1,2
        world = SubSim{};                                  // 판 비우기
        deleteChip(1);                                     // 가운데 것만 지움
        // '다' 를 판에 놓고 이름이 그대로인지 본다
        int inst = addChip(world, 2, 100, 100);
        bool nameOK = std::strcmp(compName(world.comps[inst]), "다") == 0;
        std::vector<int> lc = liveChips();
        bool listOK = lc.size() == 2 && lc[0] == 0 && lc[1] == 2;
        std::snprintf(buf, sizeof(buf), "가운데 지운 뒤 목록 %d개, 2번이 '%s'",
                      (int)lc.size(), compName(world.comps[inst]));
        check("지워도 남은 번호가 안 밀린다", nameOK && listOK, buf);
    }

    // 11. 복사하면 회로가 통째로 따라온다 (선까지). 원본이랑 안 섞인다.
    {
        world = SubSim{}; chips.clear();
        int a = addComp(SWITCH, 100, 100), b = addComp(SWITCH, 100, 200);
        int g = addComp(T_AND, 250, 140), l = addComp(LAMP, 400, 140);
        addWire(a, 0, g, 0); addWire(b, 0, g, 1); addWire(g, 0, l, 0);
        setSwitch(a, 1); setSwitch(b, 1); settle();
        bool orig = lit(world.comps[l]);                  // 원본은 켜짐

        std::vector<int> made = duplicate({ a, b, g, l }, 300, 0);
        // 복사본의 스위치를 끄면 복사본만 꺼지고 원본은 그대로여야 한다
        setSwitch(made[0], 0); settle();
        bool copyOff = !lit(world.comps[made[3]]);
        bool origStill = lit(world.comps[l]);
        // 복사본 안의 선도 살아 있는지 — 다시 켜면 다시 켜져야 한다
        setSwitch(made[0], 1); settle();
        bool copyOn = lit(world.comps[made[3]]);
        bool moved = world.comps[made[2]].x == world.comps[g].x + 300;
        std::snprintf(buf, sizeof(buf), "원본 %d, 복사본 끄면 %d·원본 %d, 다시 켜면 %d, 자리 옮김 %d",
                      (int)orig, (int)copyOff, (int)origStill, (int)copyOn, (int)moved);
        check("복사하면 선까지 따라온다", orig && copyOff && origStill && copyOn && moved, buf);
    }

    // 12. 복사할 때 밖으로 나가는 선은 안 따라온다
    //     (한쪽 끝만 고른 선까지 복제하면 원본 부품에 선이 두 개 붙는다)
    {
        world = SubSim{}; chips.clear();
        int a = addComp(SWITCH, 100, 100), l = addComp(LAMP, 400, 100);
        addWire(a, 0, l, 0);
        int before = 0; for (auto& w : world.wires) if (w.alive) ++before;
        duplicate({ a }, 0, 200);                          // 스위치만 복사
        int after = 0; for (auto& w : world.wires) if (w.alive) ++after;
        std::snprintf(buf, sizeof(buf), "선 %d개 → %d개 (안 늘어야 함)", before, after);
        check("반만 고르면 선은 안 따라온다", before == after, buf);
    }

    // 13. 돌려도 논리는 그대로고, 포트 자리는 바뀐다
    {
        world = SubSim{}; chips.clear();
        int a = addComp(SWITCH, 100, 100), b = addComp(SWITCH, 100, 200);
        int g = addComp(T_AND, 250, 140), l = addComp(LAMP, 400, 140);
        addWire(a, 0, g, 0); addWire(b, 0, g, 1); addWire(g, 0, l, 0);
        setSwitch(a, 1); setSwitch(b, 1); settle();
        bool before = lit(world.comps[l]);

        int ox0, oy0; outPort(world.comps[g], 0, ox0, oy0);
        rotateSel({ g }, 1);                               // 출력이 아래로
        int ox1, oy1; outPort(world.comps[g], 0, ox1, oy1);
        settle();
        bool after = lit(world.comps[l]);
        // 반 바퀴 더 돌리면 출력이 왼쪽
        rotateSel({ g }, 1);
        bool rotIs2 = world.comps[g].rot == 2;
        rotateSel({ g }, 2);                               // 네 번째면 제자리
        bool backTo0 = world.comps[g].rot == 0;
        std::snprintf(buf, sizeof(buf), "돌리기 전후 불 %d→%d, 포트 (%d,%d)→(%d,%d), 네 번 돌면 제자리 %d",
                      (int)before, (int)after, ox0, oy0, ox1, oy1, (int)backTo0);
        check("돌려도 논리는 그대로", before && after && (ox0 != ox1 || oy0 != oy1)
                                       && rotIs2 && backTo0, buf);
    }

    // 14. 돌리면 상자의 가로·세로가 바뀌고, 가운데는 안 움직인다
    {
        world = SubSim{}; chips.clear();
        int g = addComp(T_NAND, 300, 300);
        int w0 = compW(world.comps[g]), h0 = compH(world.comps[g]);
        int cx0 = world.comps[g].x + w0/2, cy0 = world.comps[g].y + h0/2;
        rotateSel({ g }, 1);
        int w1 = compW(world.comps[g]), h1 = compH(world.comps[g]);
        int cx1 = world.comps[g].x + w1/2, cy1 = world.comps[g].y + h1/2;
        bool swapped = (w1 == h0 && h1 == w0);
        bool centered = std::abs(cx1 - cx0) <= 1 && std::abs(cy1 - cy0) <= 1;
        std::snprintf(buf, sizeof(buf), "%dx%d → %dx%d, 가운데 (%d,%d)→(%d,%d)",
                      w0, h0, w1, h1, cx0, cy0, cx1, cy1);
        check("돌리면 가로세로가 바뀌고 가운데는 그대로", swapped && centered, buf);
    }

    // 15. 확대·이동해도 화면↔판 좌표가 서로 되돌아온다
    {
        viewZoom = 1.0f; viewX = viewY = 0;
        zoomAt(2.0f, PANEL_W + 300, 200);
        // 확대 기준점은 그 자리에 그대로 있어야 한다
        bool anchored = std::abs(w2sX(s2wX(PANEL_W + 300)) - (PANEL_W + 300)) <= 1 &&
                        std::abs(w2sY(s2wY(200)) - 200) <= 1;
        float wxv = s2wX(PANEL_W + 500), wyv = s2wY(400);
        bool roundTrip = std::abs(w2sX(wxv) - (PANEL_W + 500)) <= 1 &&
                         std::abs(w2sY(wyv) - 400) <= 1;
        viewX += 137; viewY -= 89;                        // 옮겨도 마찬가지
        float wx2 = s2wX(PANEL_W + 500), wy2 = s2wY(400);
        bool afterPan = std::abs(w2sX(wx2) - (PANEL_W + 500)) <= 1 &&
                        std::abs(w2sY(wy2) - 400) <= 1;
        // 배율 한계
        zoomAt(99.0f, PANEL_W, 0); bool capped = viewZoom <= ZOOM_MAX + 0.001f;
        zoomAt(0.001f, PANEL_W, 0); bool floored = viewZoom >= ZOOM_MIN - 0.001f;
        viewZoom = 1.0f; viewX = viewY = 0;
        std::snprintf(buf, sizeof(buf), "기준점 고정 %d, 왕복 %d, 옮긴 뒤 %d, 한계 %d/%d",
                      (int)anchored, (int)roundTrip, (int)afterPan, (int)capped, (int)floored);
        check("확대·이동 좌표가 어긋나지 않는다",
              anchored && roundTrip && afterPan && capped && floored, buf);
    }

    // 16. 저장했다 불러오면 그대로 돌아온다 (칩 · 판 · 스위치 · 보기)
    {
        world = SubSim{}; chips.clear();
        // 반가산기를 짜서 칩으로 묶고
        int a = addComp(PIN_IN, 100, 100), b = addComp(PIN_IN, 100, 200);
        int xg = addComp(T_XOR, 250, 120), ag = addComp(T_AND, 250, 220);
        int sl = addComp(PIN_OUT, 400, 120), cl = addComp(PIN_OUT, 400, 220);
        addWire(a, 0, xg, 0); addWire(b, 0, xg, 1);
        addWire(a, 0, ag, 0); addWire(b, 0, ag, 1);
        addWire(xg, 0, sl, 0); addWire(ag, 0, cl, 0);
        createChip({ a, b, xg, ag, sl, cl }, "반 가산기");   // 이름에 빈칸도 넣어 본다
        // 그 칩을 쓰는 회로를 판에 짠다
        int inst = -1;
        for (int i = 0; i < (int)world.comps.size(); ++i)
            if (world.comps[i].alive && world.comps[i].chipId >= 0) inst = i;
        int s0 = addComp(SWITCH, 600, 100), s1 = addComp(SWITCH, 600, 200);
        int o0 = addComp(LAMP, 800, 100), o1 = addComp(LAMP, 800, 200);
        addWire(s0, 0, inst, 0); addWire(s1, 0, inst, 1);
        addWire(inst, 0, o0, 0); addWire(inst, 1, o1, 0);
        world.comps[s0].rot = 2;                            // 돌린 것도 남아야 한다
        setSwitch(s0, 1); setSwitch(s1, 1);                 // 1+1 → 합 0, 올림 1
        settle();
        bool sumBefore = lit(world.comps[o0]), carBefore = lit(world.comps[o1]);
        viewZoom = 1.75f; viewX = 321; viewY = -87;

        bool wrote = saveState();
        // 싹 지우고 파일에서만 되살린다
        world = SubSim{}; chips.clear();
        viewZoom = 1.0f; viewX = viewY = 0;
        bool read = loadState();
        settle();

        bool chipOK = chips.size() == 1 && chips[0].name == "반 가산기";
        bool sumAfter = false, carAfter = false, rotOK = false;
        if (read && (int)world.comps.size() > o1) {
            sumAfter = lit(world.comps[o0]); carAfter = lit(world.comps[o1]);
            rotOK = world.comps[s0].rot == 2;
        }
        bool viewOK = std::abs(viewZoom - 1.75f) < 0.01f &&
                      std::abs(viewX - 321) < 1 && std::abs(viewY + 87) < 1;
        std::snprintf(buf, sizeof(buf),
                      "씀 %d 읽음 %d, 칩이름 %d, 합 %d→%d 올림 %d→%d, 회전 %d, 보기 %d",
                      (int)wrote, (int)read, (int)chipOK,
                      (int)sumBefore, (int)sumAfter, (int)carBefore, (int)carAfter,
                      (int)rotOK, (int)viewOK);
        check("저장했다 불러오면 그대로다",
              wrote && read && chipOK && sumAfter == sumBefore && carAfter == carBefore
              && rotOK && viewOK, buf);
    }

    // 17. 깨진 파일을 읽어도 하던 게 안 날아간다
    {
        // 16번이 쓴 멀쩡한 파일을 덮지 않게 딴 자리에 쓴다
        std::string good = savePath();
        SDL_setenv("SEMTLE_SAVE", (good + ".bad").c_str(), 1);
        world = SubSim{}; chips.clear();
        int s = addComp(SWITCH, 10, 20), l = addComp(LAMP, 200, 20);
        addWire(s, 0, l, 0);
        setSwitch(s, 1); settle();
        int keptComps = (int)world.comps.size();

        std::FILE* f = std::fopen(savePath().c_str(), "w");
        bool wrote = false;
        if (f) { std::fprintf(f, "논리 1\n칩 3\n칩시작 쓰레기\n"); std::fclose(f); wrote = true; }
        bool read = loadState();                  // 실패해야 한다
        std::snprintf(buf, sizeof(buf), "깨진 파일 읽기 %d(실패해야 함), 부품 %d개 그대로 %d, 불 %d",
                      (int)read, keptComps, (int)((int)world.comps.size() == keptComps),
                      (int)lit(world.comps[l]));
        check("깨진 파일에 안 무너진다",
              wrote && !read && (int)world.comps.size() == keptComps && lit(world.comps[l]), buf);
        std::remove(savePath().c_str());
    }

    // 18. 핀 이름이 칩의 포트 이름이 된다. 순서는 위→아래.
    {
        world = SubSim{}; chips.clear();
        int a = addComp(PIN_IN, 100, 300);            // 아래쪽에 먼저 놓고
        int b = addComp(PIN_IN, 100, 100);            // 위쪽을 나중에 놓아도
        int g = addComp(T_AND, 250, 200);
        int s = addComp(PIN_OUT, 400, 150), c2 = addComp(PIN_OUT, 400, 350);
        world.comps[a].label = "아래"; world.comps[b].label = "위";
        world.comps[s].label = "합";   world.comps[c2].label = "올림";
        addWire(b, 0, g, 0); addWire(a, 0, g, 1); addWire(g, 0, s, 0);
        createChip({ a, b, g, s, c2 }, "이름칩");

        int inst = -1;
        for (int i = 0; i < (int)world.comps.size(); ++i)
            if (world.comps[i].alive && world.comps[i].chipId >= 0) inst = i;
        const char* i0 = inst >= 0 ? portName(world.comps[inst], 0, true)  : nullptr;
        const char* i1 = inst >= 0 ? portName(world.comps[inst], 1, true)  : nullptr;
        const char* o0 = inst >= 0 ? portName(world.comps[inst], 0, false) : nullptr;
        const char* o1 = inst >= 0 ? portName(world.comps[inst], 1, false) : nullptr;
        bool ok = i0 && i1 && o0 && o1 &&
                  std::strcmp(i0, "위") == 0 && std::strcmp(i1, "아래") == 0 &&
                  std::strcmp(o0, "합") == 0 && std::strcmp(o1, "올림") == 0;
        std::snprintf(buf, sizeof(buf), "입력 [%s, %s] 출력 [%s, %s] (위→아래 순)",
                      i0?i0:"?", i1?i1:"?", o0?o0:"?", o1?o1:"?");
        check("핀 이름이 포트 이름이 된다", ok, buf);
    }

    // 19. 칩 안에 남은 스위치는 포트가 아니라 상수로 쓰인다
    //     (예전엔 안의 스위치가 몽땅 입력 포트로 끌려나가서 이게 안 됐다)
    {
        world = SubSim{}; chips.clear();
        int pin = addComp(PIN_IN, 100, 100);
        int konst = addComp(SWITCH, 100, 250);          // 켜 둔 채로 묻는다
        int g = addComp(T_AND, 250, 160);
        int out = addComp(PIN_OUT, 400, 160);
        setSwitch(konst, 1);
        addWire(pin, 0, g, 0); addWire(konst, 0, g, 1); addWire(g, 0, out, 0);
        createChip({ pin, konst, g, out }, "상수칩");

        bool onePort = chips[0].tmpl.inPorts.size() == 1 && chips[0].tmpl.outPorts.size() == 1;
        int inst = -1;
        for (int i = 0; i < (int)world.comps.size(); ++i)
            if (world.comps[i].alive && world.comps[i].chipId >= 0) inst = i;
        // 상수가 늘 1이므로 이 칩은 그냥 입력을 그대로 내보내야 한다
        int sw = addComp(SWITCH, 600, 100), lp = addComp(LAMP, 800, 100);
        addWire(sw, 0, inst, 0); addWire(inst, 0, lp, 0);
        setSwitch(sw, 1); settle(); bool hi = lit(world.comps[lp]);
        setSwitch(sw, 0); settle(); bool lo = lit(world.comps[lp]);
        std::snprintf(buf, sizeof(buf), "포트 1入1出 %d, 입력 1→%d 0→%d (묻은 스위치가 상수 1)",
                      (int)onePort, (int)hi, (int)lo);
        check("칩 안 스위치는 상수로 남는다", onePort && hi && !lo, buf);
    }

    // 20. 예전 저장본(스위치·전구가 포트)을 읽으면 핀으로 바꿔 준다
    {
        world = SubSim{}; chips.clear();
        // 손으로 예전 형식 파일을 쓴다 — 부품 줄에 이름 칸이 아예 없다
        std::FILE* f = std::fopen(savePath().c_str(), "w");
        bool wrote = false;
        if (f) {
            std::fprintf(f,
                "논리 1\n칩 1\n칩시작 5012126 1 옛칩\n"
                "부품 3\n"
                "0 -1 100 100 0 1 0\n"      // 스위치 (예전 입력 포트)
                "1 -1 300 100 0 1 0\n"      // 전구  (예전 출력 포트)
                "4 -1 200 100 0 1 0\n"      // AND (그냥 부품)
                "선 1\n0 0 1 0 1\n"
                "입력 1 0\n출력 1 1\n칩끝\n"
                "판시작\n부품 0\n선 0\n입력 0\n출력 0\n판끝\n"
                "보기 1.0000 0.0 0.0\n");
            std::fclose(f); wrote = true;
        }
        bool read = loadState();
        bool converted = read && chips.size() == 1 &&
                         chips[0].tmpl.comps.size() == 3 &&
                         chips[0].tmpl.comps[0].type == PIN_IN &&
                         chips[0].tmpl.comps[1].type == PIN_OUT &&
                         chips[0].tmpl.comps[2].type == T_AND;      // 포트 아닌 건 그대로
        bool named = converted && !chips[0].tmpl.comps[0].label.empty()
                               && !chips[0].tmpl.comps[1].label.empty();
        // 바뀐 칩이 실제로 돌아가는지
        bool works = false;
        if (converted) {
            int inst = addChip(world, 0, 100, 100);
            int sw = addComp(SWITCH, 300, 100), lp = addComp(LAMP, 500, 100);
            addWire(sw, 0, inst, 0); addWire(inst, 0, lp, 0);
            setSwitch(sw, 1); settle(); works = lit(world.comps[lp]);
        }
        std::snprintf(buf, sizeof(buf), "읽음 %d, 스위치·전구→핀 %d, 이름 붙음 %d, 돌아감 %d",
                      (int)read, (int)converted, (int)named, (int)works);
        check("예전 저장본이 핀으로 바뀐다", wrote && read && converted && named && works, buf);
        std::remove(savePath().c_str());
    }

    // 21. 되돌리기·다시하기가 판을 제자리로 돌린다
    {
        world = SubSim{}; chips.clear();
        std::vector<Snapshot> undo, redo;
        Snapshot prev = takeSnap();
        auto push = [&] {
            undo.push_back(std::move(prev));
            if ((int)undo.size() > UNDO_MAX) undo.erase(undo.begin());
            redo.clear(); prev = takeSnap();
        };
        auto undoOnce = [&] {
            if (undo.empty()) return false;
            redo.push_back(takeSnap());
            putSnap(undo.back()); undo.pop_back(); prev = takeSnap();
            return true;
        };
        auto redoOnce = [&] {
            if (redo.empty()) return false;
            undo.push_back(takeSnap());
            putSnap(redo.back()); redo.pop_back(); prev = takeSnap();
            return true;
        };

        int a = addComp(SWITCH, 100, 100); push();      // 1) 스위치
        int l = addComp(LAMP, 400, 100);   push();      // 2) 전구
        addWire(a, 0, l, 0);               push();      // 3) 선
        setSwitch(a, 1); settle();
        bool wired = lit(world.comps[l]);

        bool u1 = undoOnce(); settle();                 // 선이 없어진다
        int liveW = 0; for (auto& w : world.wires) if (w.alive) ++liveW;
        bool wireGone = (liveW == 0);
        bool u2 = undoOnce();                           // 전구가 없어진다
        int liveC = 0; for (auto& c : world.comps) if (c.alive) ++liveC;
        bool lampGone = (liveC == 1);

        bool r1 = redoOnce(), r2 = redoOnce();          // 도로 감기
        settle();
        int liveC2 = 0; for (auto& c : world.comps) if (c.alive) ++liveC2;
        int liveW2 = 0; for (auto& w : world.wires) if (w.alive) ++liveW2;
        bool back = liveC2 == 2 && liveW2 == 1;

        std::snprintf(buf, sizeof(buf), "이었을 때 %d, 되돌려 선 사라짐 %d 전구 사라짐 %d, 다시하기로 복귀 %d",
                      (int)wired, (int)wireGone, (int)lampGone, (int)back);
        check("되돌리기·다시하기", wired && u1 && u2 && wireGone && lampGone && r1 && r2 && back, buf);
    }

    // 22. 되돌려도 칩 설계도가 같이 돌아온다 (칩을 만들기 전으로)
    {
        world = SubSim{}; chips.clear();
        int p = addComp(PIN_IN, 100, 100), q = addComp(PIN_OUT, 300, 100);
        addWire(p, 0, q, 0);
        Snapshot before = takeSnap();                   // 묶기 직전
        createChip({ p, q }, "그냥통과");
        bool made = chips.size() == 1;
        putSnap(before);                                // 되돌리기
        bool chipGone = chips.empty();
        int live = 0; for (auto& c : world.comps) if (c.alive) ++live;
        std::snprintf(buf, sizeof(buf), "묶으니 칩 %d개, 되돌리니 칩 %d개·부품 %d개",
                      (int)made, (int)chips.size(), live);
        check("되돌리면 칩도 사라진다", made && chipGone && live == 2, buf);
    }

    // 23. 되돌린 스냅샷이 원본이랑 안 얽힌다 (얕게 복사하면 같이 바뀐다)
    {
        world = SubSim{}; chips.clear();
        int p = addComp(PIN_IN, 100, 100), g = addComp(T_NOT, 250, 100),
            q = addComp(PIN_OUT, 400, 100);
        addWire(p, 0, g, 0); addWire(g, 0, q, 0);
        createChip({ p, g, q }, "반전");
        Snapshot snap = takeSnap();
        // 찍어 둔 뒤에 판과 칩을 마구 흔든다
        int inst = -1;
        for (int i = 0; i < (int)world.comps.size(); ++i)
            if (world.comps[i].alive && world.comps[i].chipId >= 0) inst = i;
        if (inst >= 0) { world.comps[inst].x = 9999; deleteComp(world, inst); }
        chips[0].name = "망가뜨림";
        chips[0].tmpl.comps.clear();
        // 되돌리면 흔들기 전으로 와야 한다
        putSnap(snap);
        bool nameOK = chips.size() == 1 && chips[0].name == "반전";
        bool tmplOK = chips.size() == 1 && chips[0].tmpl.comps.size() == 3;
        int live = 0, atX = -1;
        for (auto& c : world.comps) if (c.alive) { ++live; if (c.chipId >= 0) atX = c.x; }
        std::snprintf(buf, sizeof(buf), "칩이름 '%s', 설계도 %d개, 살아있는 부품 %d개, 상자 x=%d",
                      chips.empty() ? "?" : chips[0].name.c_str(),
                      chips.empty() ? -1 : (int)chips[0].tmpl.comps.size(), live, atX);
        check("스냅샷이 원본과 안 얽힌다", nameOK && tmplOK && live == 1 && atX != 9999, buf);
    }

    // 24. 내보냈다 가져오면 그대로다 (자동 저장 파일과 딴 자리)
    {
        world = SubSim{}; chips.clear();
        int p = addComp(PIN_IN, 100, 100), g = addComp(T_XOR, 250, 100),
            q = addComp(PIN_OUT, 400, 100);
        world.comps[p].label = "들어옴"; world.comps[q].label = "나감";
        addWire(p, 0, g, 0); addWire(g, 0, q, 0);
        createChip({ p, g, q }, "내보낼칩");
        int sw = addComp(SWITCH, 600, 100);
        setSwitch(sw, 1); settle();

        std::string out = savePath() + ".export";
        bool wrote = saveTo(out, true);
        world = SubSim{}; chips.clear();                 // 싹 지우고
        bool read = loadFrom(out);
        bool ok = read && chips.size() == 1 && chips[0].name == "내보낼칩" &&
                  chips[0].tmpl.inPorts.size() == 1 &&
                  chips[0].tmpl.comps[chips[0].tmpl.inPorts[0]].label == "들어옴";
        // 자동 저장 파일은 안 건드려야 한다
        std::FILE* auto_ = std::fopen(savePath().c_str(), "r");
        bool untouched = (auto_ == nullptr);
        if (auto_) std::fclose(auto_);
        std::snprintf(buf, sizeof(buf), "내보냄 %d 가져옴 %d, 칩·핀이름 %d, 자동저장 파일 안 건드림 %d",
                      (int)wrote, (int)read, (int)ok, (int)untouched);
        check("내보냈다 가져오면 그대로다", wrote && read && ok && untouched, buf);
        std::remove(out.c_str());
    }

    // 25. 채점기가 맞는 회로는 통과, 틀린 회로는 잡아낸다
    {
        int LN = lessonByName("NOT짜기");            // NAND 로 NOT 만드는 단계
        const Lesson& L = LESSONS[LN];
        char why[160];

        // (가) 시작 판만 있고 아무것도 안 이었으면 통과하면 안 된다
        screen = SC_LEARN; chips.clear();
        setupLesson(LN);
        bool emptyFails = !gradeLesson(L, why, sizeof(why));

        // (나) NAND 두 입력에 같은 것을 넣으면 NOT 이 된다 → 통과해야 한다
        setupLesson(LN);
        int a = findPin(PIN_IN, L.inName[0]), z = findPin(PIN_OUT, L.outName[0]);
        int g = addComp(world, T_NAND, 300, 150);
        addWire(world, a, 0, g, 0); addWire(world, a, 0, g, 1);
        addWire(world, g, 0, z, 0);
        bool rightPasses = gradeLesson(L, why, sizeof(why));

        // (다) 일부러 틀리게 — NAND 대신 그냥 이어 버리면 잡아야 한다
        setupLesson(LN);
        a = findPin(PIN_IN, L.inName[0]); z = findPin(PIN_OUT, L.outName[0]);
        addWire(world, a, 0, z, 0);
        bool wrongFails = !gradeLesson(L, why, sizeof(why));
        bool tellsWhy = std::strstr(why, L.outName[0]) != nullptr;   // 어디가 틀렸는지 말해 준다

        // (라) 핀 이름이 다르면 못 찾는다고 해야 한다
        setupLesson(LN);
        world.comps[findPin(PIN_IN, L.inName[0])].label = "엉뚱";
        char why2[160];
        bool noPin = !gradeLesson(L, why2, sizeof(why2)) && std::strstr(why2, "입력핀") != nullptr;

        std::snprintf(buf, sizeof(buf), "빈판 막힘 %d, 맞으면 통과 %d, 틀리면 막힘 %d(이유 %d), 핀없음 %d",
                      (int)emptyFails, (int)rightPasses, (int)wrongFails, (int)tellsWhy, (int)noPin);
        check("채점기가 맞고 틀림을 가린다",
              emptyFails && rightPasses && wrongFails && tellsWhy && noPin, buf);
        screen = SC_SANDBOX;
    }

    // 26. 통과하면 그 회로가 칩이 되고, 판은 그대로 남는다
    {
        int LN = lessonByName("NOT짜기");
        const Lesson& L = LESSONS[LN];
        screen = SC_LEARN; chips.clear();
        setupLesson(LN);
        int a = findPin(PIN_IN, L.inName[0]), z = findPin(PIN_OUT, L.outName[0]);
        int g = addComp(world, T_NAND, 300, 150);
        addWire(world, a, 0, g, 0); addWire(world, a, 0, g, 1);
        addWire(world, g, 0, z, 0);
        int before = 0; for (auto& c : world.comps) if (c.alive) ++before;

        bankLesson(L);
        bool made = chips.size() == 1 && chips[0].name == L.name;
        int after = 0; for (auto& c : world.comps) if (c.alive) ++after;
        bool boardKept = (after == before);

        bankLesson(L);                              // 두 번 눌러도 안 늘어난다
        bool noDup = chips.size() == 1;

        // 얻은 칩이 실제로 NOT 으로 돌아야 한다
        screen = SC_SANDBOX; world = SubSim{};
        int inst = addChip(world, 0, 100, 100);
        int sw = addComp(world, SWITCH, 300, 100), lp = addComp(world, LAMP, 500, 100);
        addWire(world, sw, 0, inst, 0); addWire(world, inst, 0, lp, 0);
        setSwitch(sw, 0); settle(); bool off = lit(world.comps[lp]);
        setSwitch(sw, 1); settle(); bool on  = lit(world.comps[lp]);

        std::snprintf(buf, sizeof(buf), "칩 생김 %d, 판 %d→%d개 그대로 %d, 두 번 눌러도 %d개, 0→%d 1→%d",
                      (int)made, before, after, (int)boardKept, (int)chips.size(), (int)off, (int)on);
        check("통과하면 칩이 되고 판은 남는다",
              made && boardKept && noDup && off && !on, buf);
    }

    // 27. 이 단계에서 못 쓰는 부품은 막힌다
    {
        screen = SC_LEARN; chips.clear();
        lessonAt = lessonByTitle("AND —");           // AND 하나만 쓰는 단계
        bool nandOK = false, xorBlocked = false, pinOK = false;
        for (int i = 1; i <= TYPE_N; ++i) {
            if (TYPES[i-1].name == std::string("AND")) nandOK = toolEnabled(i);
            if (TYPES[i-1].name == std::string("XOR")) xorBlocked = !toolEnabled(i);
            if (i - 1 == PIN_IN)                       pinOK = toolEnabled(i);
        }
        screen = SC_SANDBOX;
        bool allFreeInSandbox = true;
        for (int i = 1; i <= TYPE_N; ++i) if (!toolEnabled(i)) allFreeInSandbox = false;
        std::snprintf(buf, sizeof(buf), "AND단계에서 AND %d · XOR 막힘 %d · 핀 %d, 샌드박스는 다 열림 %d",
                      (int)nandOK, (int)xorBlocked, (int)pinOK, (int)allFreeInSandbox);
        check("단계마다 쓸 부품이 제한된다",
              nandOK && xorBlocked && pinOK && allFreeInSandbox, buf);
    }

    // 28. 학습과 샌드박스는 저장본이 따로다
    {
        std::string ls = savePathFor(SC_LEARN), ss = savePathFor(SC_SANDBOX);
        bool differ = (ls != ss);

        // 샌드박스에 스위치 하나
        screen = SC_SANDBOX; world = SubSim{}; chips.clear();
        addComp(world, SWITCH, 11, 22);
        lessonAt = 0; lessonDone = 0;
        bool w1 = saveState();

        // 학습에 전혀 다른 판 + 진도
        screen = SC_LEARN; world = SubSim{}; chips.clear();
        for (int i = 0; i < 5; ++i) addComp(world, T_NOR, 100 + i*40, 300);
        lessonAt = 3; lessonDone = 4;
        bool w2 = saveState();

        // 샌드박스를 다시 읽으면 스위치 하나짜리 그대로여야 한다
        screen = SC_SANDBOX; world = SubSim{}; chips.clear(); lessonAt = lessonDone = 0;
        bool r1 = loadState();
        int n1 = 0; for (auto& c : world.comps) if (c.alive) ++n1;
        bool sandOK = r1 && n1 == 1 && world.comps[0].type == SWITCH && world.comps[0].x == 11;

        // 학습도 그대로
        screen = SC_LEARN; world = SubSim{}; chips.clear();
        bool r2 = loadState();
        int n2 = 0; for (auto& c : world.comps) if (c.alive) ++n2;
        bool learnOK = r2 && n2 == 5 && lessonAt == 3 && lessonDone == 4;

        std::snprintf(buf, sizeof(buf), "경로 다름 %d, 샌드박스 %d개 %d, 학습 %d개·진도 %d/%d %d",
                      (int)differ, n1, (int)sandOK, n2, lessonAt, lessonDone, (int)learnOK);
        check("모드끼리 저장본이 안 섞인다", differ && w1 && w2 && sandOK && learnOK, buf);
        std::remove(ls.c_str()); std::remove(ss.c_str());
        screen = SC_SANDBOX; lessonAt = lessonDone = 0;
    }

    // 29. 모든 단계를 앞에서부터 실제로 풀 수 있다
    //     — 허락된 부품만으로 정답 회로를 짜서 채점기에 넣어 본다.
    //     한 단계라도 못 풀면 학습 모드가 거기서 막힌다. 제일 중요한 검사다.
    {
        screen = SC_LEARN; chips.clear();
        char why[160] = ""; int cleared = 0; char detail[200] = "";

        bool broke = false;

        for (int at = 0; at < LESSON_N && !broke; ++at) {
            const Lesson& L = LESSONS[at];
            lessonAt = at;
            setupLesson(at);
            if (!buildSolution(at, detail, sizeof(detail))) { broke = true; break; }

            // 정답이 그 단계의 규칙을 지켰는지도 본다
            for (const auto& c : world.comps) {
                if (!c.alive) continue;
                const char* n = (c.chipId >= 0) ? chips[c.chipId].name.c_str() : TYPES[c.type].name;
                bool free_ = (c.chipId < 0) && (isPin(c.type) || c.type == SWITCH || c.type == LAMP);
                if (!free_ && !lessonAllows(L, n)) {
                    std::snprintf(detail, sizeof(detail), "%d단계: 정답에 못 쓰는 '%s' 가 들어감", at+1, n);
                    broke = true; break;
                }
            }
            if (broke) break;

            if (!gradeLesson(L, why, sizeof(why))) {
                std::snprintf(detail, sizeof(detail), "%d단계(%s) 못 품 — %s", at + 1, L.title, why);
                break;
            }
            bankLesson(L);
            ++cleared;
        }
        if (cleared == LESSON_N)
            std::snprintf(detail, sizeof(detail), "%d단계 다 풀림 (칩 %d개 얻음)",
                          LESSON_N, (int)chips.size());
        check("학습 단계가 순서대로 풀린다", cleared == LESSON_N, detail);
        screen = SC_SANDBOX; lessonAt = 0; chips.clear(); world = SubSim{};
    }

    // 30. 과정을 고치면 진도만 처음으로 돌아가고 얻은 칩은 남는다
    {
        screen = SC_LEARN;
        world = SubSim{}; chips.clear();
        // 칩 하나 얻어 둔 상태를 만든다
        int p = addComp(world, PIN_IN, 100, 100), q = addComp(world, PIN_OUT, 300, 100);
        addWire(world, p, 0, q, 0);
        createChip({ p, q }, "얻은칩");
        lessonAt = 5; lessonDone = 6;
        bool wrote = saveState();

        // (가) 같은 과정 번호면 진도가 그대로
        lessonAt = lessonDone = 0; chips.clear(); world = SubSim{};
        bool r1 = loadState();
        bool kept = r1 && lessonAt == 5 && lessonDone == 6 && chips.size() == 1;

        // (나) 파일의 과정 번호를 옛것으로 바꿔 놓으면 진도만 초기화된다
        {
            std::FILE* f = std::fopen(savePath().c_str(), "r");
            std::string all; char ch;
            while (f && std::fread(&ch, 1, 1, f) == 1) all += ch;
            if (f) std::fclose(f);
            char oldLine[48], newLine[48];
            std::snprintf(oldLine, sizeof(oldLine), "진도 5 6 %d", COURSE_VER);
            std::snprintf(newLine, sizeof(newLine), "진도 5 6 %d", COURSE_VER - 1);
            size_t at = all.find(oldLine);
            if (at != std::string::npos) all.replace(at, std::strlen(oldLine), newLine);
            f = std::fopen(savePath().c_str(), "w");
            if (f) { std::fwrite(all.data(), 1, all.size(), f); std::fclose(f); }
        }
        lessonAt = 3; lessonDone = 3; chips.clear(); world = SubSim{};
        bool r2 = loadState();
        bool reset = r2 && lessonAt == 0 && lessonDone == 0;
        bool chipsKept = chips.size() == 1 && chips[0].name == "얻은칩";

        // (다) 과정 번호가 아예 없던 시절 파일도 초기화되어야 한다 (실제로 그런 파일이 있었다)
        {
            std::FILE* f = std::fopen(savePath().c_str(), "r");
            std::string all; char ch;
            while (f && std::fread(&ch, 1, 1, f) == 1) all += ch;
            if (f) std::fclose(f);
            char cur[48]; std::snprintf(cur, sizeof(cur), "진도 0 0 %d", COURSE_VER - 1);
            size_t at = all.find("진도 ");
            if (at != std::string::npos) {
                size_t e = all.find('\n', at);
                all.replace(at, e - at, "진도 4 5");       // 번호 없는 옛 형식
            }
            (void)cur;
            f = std::fopen(savePath().c_str(), "w");
            if (f) { std::fwrite(all.data(), 1, all.size(), f); std::fclose(f); }
        }
        lessonAt = 7; lessonDone = 7; chips.clear(); world = SubSim{};
        bool r3 = loadState();
        bool oldReset = r3 && lessonAt == 0 && lessonDone == 0 && chips.size() == 1;

        std::snprintf(buf, sizeof(buf), "같은 과정 5/6 유지 %d, 번호 다르면 초기화 %d, 번호 없어도 초기화 %d, 칩 남음 %d",
                      (int)kept, (int)reset, (int)oldReset, (int)chipsKept);
        check("과정이 바뀌면 진도만 초기화된다", wrote && kept && reset && chipsKept && oldReset, buf);
        std::remove(savePath().c_str());
        screen = SC_SANDBOX; lessonAt = lessonDone = 0;
    }

    // 31. 어느 단계든 설명판 내용이 단추를 안 밀어낸다 (배율·창 크기를 다 바꿔 가며)
    {
        float ous = uiScale;
        int worstAt = -1; float worstUs = 0; int worstOver = 0, smallest = 99;
        for (float us : { 0.8f, 1.0f, 1.35f, 1.7f, 2.2f }) {
            uiScale = us; applyUiScale();
            for (int i = 0; i < LESSON_N; ++i) {
                LesFit f = lessonFit(LESSONS[i], WIN_H_MIN);
                if (f.need > f.room && f.need - f.room > worstOver) {
                    worstOver = f.need - f.room; worstAt = i; worstUs = us;
                }
                smallest = std::min(smallest, f.ts);
            }
        }
        uiScale = ous; applyUiScale();
        if (worstAt < 0)
            std::snprintf(buf, sizeof(buf), "배율 0.8~2.2 어디서나 다 들어감 (제일 작은 글자 %d)", smallest);
        else
            std::snprintf(buf, sizeof(buf), "배율 %.1f 의 %d단계가 %dpx 넘침", worstUs, worstAt + 1, worstOver);
        // 대본이 긴 단계는 9까지 줄어든다 (2.2배에서 실제 20px 라 읽을 만하다)
        check("설명판이 작은 창에서도 안 넘친다", worstAt < 0 && smallest >= 9, buf);
    }

    // 32. 창 크기를 바꿔도 판 자리 계산이 안 어긋난다
    {
        int ow = WIN_W, oh = WIN_H; float ous = uiScale;
        bool ok = true; char detail[160] = "";
        // 배율마다 그 배율에서 나오는 창 크기들을 함께 본다
        struct Case { float us; int w, h; };
        std::vector<Case> sizes;
        for (float us : { 0.8f, 1.35f, 2.2f }) {
            uiScale = us; applyUiScale();
            sizes.push_back({ us, WIN_W_MIN, WIN_H_MIN });
            sizes.push_back({ us, WIN_W0, WIN_H0 });
            sizes.push_back({ us, std::max(1600, WIN_W_MIN), std::max(900, WIN_H_MIN) });
        }
        for (auto& c : sizes) {
            int sz[2] = { c.w, c.h };
            uiScale = c.us; applyUiScale();
            WIN_W = sz[0]; WIN_H = sz[1];
            for (int sc = SC_LEARN; sc <= SC_SANDBOX; ++sc) {
                screen = sc;
                if (canvasR() - canvasL() < 200) {
                    ok = false;
                    std::snprintf(detail, sizeof(detail), "%dx%d %s 에서 판이 %d픽셀뿐 (배율 %.1f)",
                                  sz[0], sz[1], sc == SC_LEARN ? "학습" : "샌드박스",
                                  canvasR() - canvasL(), uiScale);
                }
                // 화면↔판 좌표가 여전히 왕복하는지
                viewZoom = 1.5f; viewX = 40; viewY = -30;
                int sx = (canvasL() + canvasR()) / 2, sy = WIN_H / 2;
                if (std::abs(w2sX(s2wX(sx)) - sx) > 1 || std::abs(w2sY(s2wY(sy)) - sy) > 1) {
                    ok = false;
                    std::snprintf(detail, sizeof(detail), "%dx%d 에서 좌표가 어긋남", sz[0], sz[1]);
                }
            }
            // 첫 화면 단추가 창 안에 들어오는지
            for (int i = 0; i < 2; ++i) {
                Rect r = menuBtn(i);
                if (r.x < 0 || r.x + r.w > WIN_W || r.y < 0 || r.y + r.h > WIN_H) {
                    ok = false;
                    std::snprintf(detail, sizeof(detail), "%dx%d 에서 첫 화면 단추가 창 밖", sz[0], sz[1]);
                }
            }
        }
        uiScale = ous; applyUiScale();
        WIN_W = ow; WIN_H = oh; screen = SC_SANDBOX;
        viewZoom = 1.0f; viewX = viewY = 0;
        check("창 크기가 바뀌어도 자리가 안 어긋난다", ok,
              ok ? "배율 셋 × 크기 셋, 아홉 가지 다 괜찮음" : detail);
    }

    // 33. 칩이 많아도 부품 목록이 다 보인다 (줄을 좁혀서)
    {
        int ow = WIN_W, oh = WIN_H; float ous = uiScale;
        bool everFail = false; char why2[120] = "";
        for (float us : { 0.8f, 1.35f, 2.2f }) {
        uiScale = us; applyUiScale();
        WIN_W = WIN_W_MIN; WIN_H = WIN_H_MIN;       // 제일 빡빡한 창
        screen = SC_SANDBOX; world = SubSim{}; chips.clear();
        // 학습을 다 끝내면 얻는 만큼 칩을 만든다
        for (int i = 0; i < 6; ++i) {
            int p2 = addComp(world, PIN_IN, 100, 100), q2 = addComp(world, PIN_OUT, 300, 100);
            addWire(world, p2, 0, q2, 0);
            char n[16]; std::snprintf(n, sizeof(n), "칩%d", i + 1);
            createChip({ p2, q2 }, n);
        }
        int nt = shownCount();
        // 끝까지 굴리면 마지막 줄이 자리 안에 들어와야 한다
        toolScroll = toolScrollMax();
        Rect last = toolRect(nt - 1);
        if (last.y + last.h > toolBot() + S(2)) {
            everFail = true;
            std::snprintf(why2, sizeof(why2), "배율 %.1f (%dx%d): 끝까지 굴려도 마지막 줄 %d > %d",
                          us, WIN_W, WIN_H, last.y + last.h, toolBot());
        }
        // 맨 위로 굴리면 첫 줄이 보여야 한다
        toolScroll = 0;
        if (toolRect(0).y < toolTop() - S(2)) {
            everFail = true;
            std::snprintf(why2, sizeof(why2), "배율 %.1f: 맨 위로 굴려도 첫 줄이 안 보임", us);
        }
        // 굴림 값이 한도를 안 넘는다
        toolScroll = 99999; clampToolScroll();
        if (toolScroll != toolScrollMax()) {
            everFail = true;
            std::snprintf(why2, sizeof(why2), "배율 %.1f: 굴림 한도가 안 먹음", us);
        }
        if (us > 2.0f)
            std::snprintf(buf, sizeof(buf), "배율 0.8~2.2 에서 도구 %d개, 2.2배 줄높이 %d·굴릴 거리 %dpx",
                          nt, toolPitch(), toolScrollMax());
        toolScroll = 0;
        }
        uiScale = ous; applyUiScale();
        check("칩이 많아도 목록에 다 닿는다", !everFail, everFail ? why2 : buf);
        WIN_W = ow; WIN_H = oh; world = SubSim{}; chips.clear();
    }

    // 34. 단계마다 힌트가 있고, 답을 통째로 알려 주지는 않는다
    {
        // 제일 좁은 창에서 접었을 때도 판을 다 덮지 않아야 한다
        int ow = WIN_W, oh = WIN_H, os = screen;
        WIN_W = WIN_W_MIN; WIN_H = WIN_H_MIN; screen = SC_LEARN;
        int maxW = canvasR() - canvasL() - 24 - 40;
        int missing = -1, tooTall = -1, worst = 0, over = -1;
        for (int i = 0; i < LESSON_N; ++i) {
            const char* h = LESSONS[i].hint;
            if (!h || !*h) { missing = i; break; }
            std::vector<std::string> ls = hintLines(h, 15, maxW);
            worst = std::max(worst, (int)ls.size());
            if ((int)ls.size() > 7) tooTall = i;                  // 판 절반을 넘게 덮는다
            for (auto& l : ls)
                if (textWidth(15, l.c_str()) > maxW) over = i;    // 접었는데도 삐져나옴
        }
        int lo = WIN_W, lh2 = WIN_H;
        WIN_W = ow; WIN_H = oh; screen = os;
        if (missing >= 0)      std::snprintf(buf, sizeof(buf), "%d단계에 힌트가 없다", missing + 1);
        else if (over >= 0)    std::snprintf(buf, sizeof(buf), "%d단계 힌트가 접어도 삐져나옴", over + 1);
        else if (tooTall >= 0) std::snprintf(buf, sizeof(buf), "%d단계 힌트가 너무 길다", tooTall + 1);
        else std::snprintf(buf, sizeof(buf), "%d단계 다 있음 (제일 좁은 창 %dx%d 에서 최대 %d줄)",
                           LESSON_N, lo, lh2, worst);
        check("단계마다 힌트가 있다", missing < 0 && tooTall < 0 && over < 0, buf);
    }

    // 35. 나갈까 묻는 창의 단추가 창 안에 들어온다 (어느 크기에서든)
    {
        int ow = WIN_W, oh = WIN_H;
        bool ok = true; char detail[120] = "";
        int sizes[][2] = { { WIN_W_MIN, WIN_H_MIN }, { 1600, 900 }, { WIN_W0, WIN_H0 } };
        for (auto& sz : sizes) {
            WIN_W = sz[0]; WIN_H = sz[1];
            Rect b = confirmBox(), y = confirmYes(), n = confirmNo();
            auto inside = [&](Rect r, Rect o) {
                return r.x >= o.x && r.y >= o.y && r.x + r.w <= o.x + o.w && r.y + r.h <= o.y + o.h;
            };
            Rect win{ 0, 0, WIN_W, WIN_H };
            if (!inside(b, win) || !inside(y, b) || !inside(n, b) ||
                (y.x + y.w > n.x)) {                       // 두 단추가 안 겹쳐야 한다
                ok = false;
                std::snprintf(detail, sizeof(detail), "%dx%d 에서 확인창 자리가 어긋남", sz[0], sz[1]);
            }
        }
        WIN_W = ow; WIN_H = oh;
        check("나갈까 묻는 창이 제자리에 뜬다", ok, ok ? "최소·큰창·처음크기 다 괜찮음" : detail);
    }

    // 36. 글 접기가 좁은 자리에서도 삐져나오지 않고 글자를 안 잃는다
    {
        const char* src =
            "반가산기 둘을 잇는다. 첫째에 A,B 를 넣고\n"
            "둘째에 첫째의 합과 올림입력을 넣는다. 아주아주아주아주긴한덩어리도있다";
        bool ok = true; char detail[160] = "";
        for (int w : { 90, 140, 220, 400 }) {
            std::vector<std::string> ls = hintLines(src, 15, w);
            // (가) 어느 줄도 자리를 넘지 않는다
            for (auto& l : ls)
                if (textWidth(15, l.c_str()) > w && l.size() > 3) {
                    ok = false;
                    std::snprintf(detail, sizeof(detail), "너비 %d 에서 '%s' 가 %dpx 로 넘침",
                                  w, l.c_str(), textWidth(15, l.c_str()));
                }
            // (나) 글자가 사라지거나 늘어나지 않는다 (빈칸·줄바꿈 빼고)
            std::string a2, b2 = src;
            for (auto& l : ls) a2 += l;
            auto strip = [](std::string t) {
                std::string r;
                for (char c : t) if (c != ' ' && c != '\n') r += c;
                return r;
            };
            if (strip(a2) != strip(b2)) {
                ok = false;
                std::snprintf(detail, sizeof(detail), "너비 %d 에서 글자가 달라짐 (%d→%d바이트)",
                              w, (int)strip(b2).size(), (int)strip(a2).size());
            }
        }
        std::vector<std::string> narrow = hintLines(src, 15, 90);
        if (ok) std::snprintf(buf, sizeof(buf),
                              "네 너비에서 안 넘치고 글자도 그대로 (90px 에선 %d줄)", (int)narrow.size());
        check("글 접기가 자리에 맞춘다", ok, ok ? buf : detail);
    }

    // 37. 글자 배율이 실제로 글자와 판을 키우고, 저장됐다 돌아온다
    {
        float ous = uiScale; int ow = WIN_W, oh = WIN_H;

        uiScale = 1.0f; applyUiScale();
        int w1 = textWidth(15, "가나다라"), p1 = PANEL_W, l1 = LES_W, t1 = toolRect(0).h;
        uiScale = 2.0f; applyUiScale();
        int w2 = textWidth(15, "가나다라"), p2 = PANEL_W, l2 = LES_W, t2 = toolRect(0).h;

        // 두 배로 하면 대략 두 배가 되어야 한다 (반올림 오차는 봐준다)
        bool textGrew  = w2 > w1 * 17 / 10 && w2 < w1 * 23 / 10;
        bool panelGrew = p2 == p1 * 2 && l2 == l1 * 2;
        bool rowGrew   = t2 > t1 * 15 / 10;

        // 한계 밖으로 못 나간다
        uiScale = 9.0f;  applyUiScale(); bool capped  = uiScale <= UI_MAX + 0.001f;
        uiScale = 0.05f; applyUiScale(); bool floored = uiScale >= UI_MIN - 0.001f;

        // 저장했다 불러오면 그대로
        uiScale = 1.7f; applyUiScale();
        screen = SC_SANDBOX; world = SubSim{}; chips.clear();
        addComp(world, SWITCH, 10, 10);
        bool wrote = saveState();
        uiScale = 1.0f; applyUiScale();
        bool read = loadState();
        bool kept = read && std::abs(uiScale - 1.7f) < 0.01f && PANEL_W == S(PANEL_W0);

        uiScale = ous; applyUiScale(); WIN_W = ow; WIN_H = oh;
        std::remove(savePath().c_str());
        std::snprintf(buf, sizeof(buf),
                      "글자 %d→%dpx, 판 %d→%d, 줄 %d→%d, 한계 %d/%d, 저장 %d",
                      w1, w2, p1, p2, t1, t2, (int)capped, (int)floored, (int)kept);
        check("글자 배율이 다 같이 커진다",
              textGrew && panelGrew && rowGrew && capped && floored && wrote && kept, buf);
    }

    // 38. Tab 으로 판을 숨기면 판이 화면 전체가 된다
    {
        int ow = WIN_W, oh = WIN_H; bool ou = uiOn; int os = screen;
        WIN_W = WIN_W0; WIN_H = WIN_H0;
        bool ok = true; char detail[160] = "";

        for (int sc = SC_LEARN; sc <= SC_SANDBOX; ++sc) {
            screen = sc;
            uiOn = true;
            int onL = canvasL(), onR = canvasR(), onW = onR - onL;
            uiOn = false;
            int offL = canvasL(), offR = canvasR(), offW = offR - offL;

            if (offL != 0 || offR != WIN_W) {
                ok = false;
                std::snprintf(detail, sizeof(detail), "%s 에서 숨겨도 판이 %d~%d",
                              sc == SC_LEARN ? "학습" : "샌드박스", offL, offR);
            }
            if (offW <= onW) {
                ok = false;
                std::snprintf(detail, sizeof(detail), "%s 에서 숨겼는데 판이 안 넓어짐 (%d→%d)",
                              sc == SC_LEARN ? "학습" : "샌드박스", onW, offW);
            }
            // 숨긴 상태에서도 화면↔판 좌표가 왕복해야 한다
            viewZoom = 1.4f; viewX = 55; viewY = -20;
            for (int sx : { 0, WIN_W / 2, WIN_W - 1 })
                if (std::abs(w2sX(s2wX(sx)) - sx) > 1) {
                    ok = false;
                    std::snprintf(detail, sizeof(detail), "숨긴 상태 x=%d 에서 좌표가 어긋남", sx);
                }
        }
        // 학습 모드에서 숨기면 설명판 너비만큼 더 넓어진다
        screen = SC_LEARN; uiOn = true;  int a1 = canvasR() - canvasL();
        uiOn = false; int a2 = canvasR() - canvasL();
        bool gained = (a2 - a1 == PANEL_W + LES_W);

        uiOn = ou; screen = os; WIN_W = ow; WIN_H = oh;
        viewZoom = 1.0f; viewX = viewY = 0;
        if (ok) std::snprintf(buf, sizeof(buf), "학습에서 판이 %d→%dpx (도구판+설명판 %d만큼)",
                              a1, a2, PANEL_W + LES_W);
        check("Tab 으로 판을 숨기면 넓어진다", ok && gained, ok ? buf : detail);
    }

    // 43. 이름 바꾸기 전 저장본('논리' 머리말)도 그대로 읽힌다
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear(); editStack.clear();
        std::FILE* f = std::fopen(savePath().c_str(), "w");
        bool wrote = false;
        if (f) {
            std::fprintf(f,
                "논리 1\n칩 1\n칩시작 5012126 1 옛이름칩\n"
                "부품 2\n2 -1 100 100 0 1 0 들어옴\n3 -1 300 100 0 1 0 나감\n"
                "선 1\n0 0 1 0 1\n입력 1 0\n출력 1 1\n칩끝\n"
                "판시작\n부품 1\n0 -1 50 60 0 1 1 \n선 0\n입력 0\n출력 0\n판끝\n"
                "보기 1.0000 0.0 0.0\n");
            std::fclose(f); wrote = true;
        }
        bool readOld = loadState();
        bool chipOK = readOld && chips.size() == 1 && chips[0].name == "옛이름칩";
        int live = 0; for (auto& c : world.comps) if (c.alive) ++live;

        // 다시 저장하면 새 머리말('셈틀') 로 쓴다
        bool wrote2 = saveState();
        char head[32] = "";
        f = std::fopen(savePath().c_str(), "r");
        if (f) { if (std::fscanf(f, " %31s", head) != 1) head[0] = 0; std::fclose(f); }
        bool newHead = (std::strcmp(head, "셈틀") == 0);
        // 새 머리말도 당연히 읽힌다
        chips.clear(); world = SubSim{};
        bool readNew = loadState() && chips.size() == 1;

        std::snprintf(buf, sizeof(buf), "옛 파일 읽음 %d(칩 %d·부품 %d개), 다시 쓰면 머리말 '%s' %d, 새 파일도 읽힘 %d",
                      (int)readOld, (int)chipOK, live, head, (int)newHead, (int)readNew);
        check("이름 바꾸기 전 저장본도 읽힌다",
              wrote && readOld && chipOK && live == 1 && wrote2 && newHead && readNew, buf);
        std::remove(savePath().c_str());
    }

    // 39. 칩을 고치면 이미 놓인 상자도 다 같이 바뀐다
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear(); editStack.clear();
        // AND 로 도는 칩 하나
        int p2 = addComp(world, PIN_IN, 100, 100), q2 = addComp(world, PIN_IN, 100, 220);
        int g  = addComp(world, T_AND, 250, 160);
        int o  = addComp(world, PIN_OUT, 400, 160);
        addWire(world, p2, 0, g, 0); addWire(world, q2, 0, g, 1); addWire(world, g, 0, o, 0);
        createChip({ p2, q2, g, o }, "고칠칩");

        // 그 칩을 두 군데 놓고 이어 둔다
        world = SubSim{};
        int i1 = addChip(world, 0, 100, 100), i2 = addChip(world, 0, 100, 400);
        int s1 = addComp(world, SWITCH, 20, 100), s2 = addComp(world, SWITCH, 20, 200);
        int l1 = addComp(world, LAMP, 300, 100), l2 = addComp(world, LAMP, 300, 400);
        addWire(world, s1, 0, i1, 0); addWire(world, s2, 0, i1, 1); addWire(world, i1, 0, l1, 0);
        addWire(world, s1, 0, i2, 0); addWire(world, s2, 0, i2, 1); addWire(world, i2, 0, l2, 0);
        setSwitch(s1, 1); setSwitch(s2, 0); settle();
        bool andBefore = !lit(world.comps[l1]) && !lit(world.comps[l2]);   // 1 AND 0 = 0

        // 속으로 들어가 AND 를 OR 로 바꾼다
        bool opened = beginEdit(0);
        int gi = -1;
        for (int i = 0; i < (int)world.comps.size(); ++i)
            if (world.comps[i].alive && world.comps[i].type == T_AND) gi = i;
        bool found = (gi >= 0);
        if (found) world.comps[gi].type = T_OR;
        int cut = endEdit();
        settle();
        bool orAfter = lit(world.comps[l1]) && lit(world.comps[l2]);       // 1 OR 0 = 1

        std::snprintf(buf, sizeof(buf), "열림 %d, AND일 때 둘 다 꺼짐 %d, OR로 고치니 둘 다 켜짐 %d, 끊긴 선 %d",
                      (int)opened, (int)andBefore, (int)orAfter, cut);
        check("칩을 고치면 놓인 것도 다 바뀐다", opened && found && andBefore && orAfter && cut == 0, buf);
    }

    // 40. 칩 속의 칩까지 내려간다 (겉칩이 속칩 고침을 받는다)
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear(); editStack.clear();
        // 속칩: 그냥 통과
        int a2 = addComp(world, PIN_IN, 100, 100), b2 = addComp(world, PIN_OUT, 300, 100);
        addWire(world, a2, 0, b2, 0);
        createChip({ a2, b2 }, "속칩");
        // 겉칩: 속칩을 하나 품는다
        world = SubSim{};
        int pi = addComp(world, PIN_IN, 100, 100);
        int inner = addChip(world, 0, 250, 100);
        int po = addComp(world, PIN_OUT, 450, 100);
        addWire(world, pi, 0, inner, 0); addWire(world, inner, 0, po, 0);
        createChip({ pi, inner, po }, "겉칩");
        // 판에 겉칩만 놓는다
        world = SubSim{};
        int outer = addChip(world, 1, 100, 100);
        int sw = addComp(world, SWITCH, 20, 100), lp = addComp(world, LAMP, 300, 100);
        addWire(world, sw, 0, outer, 0); addWire(world, outer, 0, lp, 0);
        setSwitch(sw, 1); settle();
        bool passBefore = lit(world.comps[lp]);

        // 속칩을 '뒤집기' 로 바꾼다 → 겉칩도 뒤집기가 되어야 한다
        beginEdit(0);
        int ai = -1, bi = -1;
        for (int i = 0; i < (int)world.comps.size(); ++i) {
            if (!world.comps[i].alive) continue;
            if (world.comps[i].type == PIN_IN)  ai = i;
            if (world.comps[i].type == PIN_OUT) bi = i;
        }
        for (auto& w : world.wires) w.alive = false;              // 있던 선 걷어내고
        int nt = addComp(world, T_NOT, 200, 100);
        addWire(world, ai, 0, nt, 0); addWire(world, nt, 0, bi, 0);
        endEdit();
        settle();
        bool flipped = !lit(world.comps[lp]);                     // 1 → 0
        setSwitch(sw, 0); settle();
        bool flipped2 = lit(world.comps[lp]);                     // 0 → 1

        std::snprintf(buf, sizeof(buf), "고치기 전 통과 %d, 속칩 뒤집으니 겉칩도 1→%d 0→%d",
                      (int)passBefore, (int)!flipped, (int)flipped2);
        check("칩 속의 칩까지 고침이 내려간다", passBefore && flipped && flipped2, buf);
    }

    // 41. 포트를 줄이면 없어진 포트에 걸린 선만 끊긴다
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear(); editStack.clear();
        int i0 = addComp(world, PIN_IN, 100, 100), i1b = addComp(world, PIN_IN, 100, 250);
        int g  = addComp(world, T_OR, 250, 170);
        int o0 = addComp(world, PIN_OUT, 400, 100), o1 = addComp(world, PIN_OUT, 400, 250);
        addWire(world, i0, 0, g, 0); addWire(world, i1b, 0, g, 1);
        addWire(world, g, 0, o0, 0); addWire(world, g, 0, o1, 0);
        createChip({ i0, i1b, g, o0, o1 }, "두갈래");            // 입력2 출력2

        world = SubSim{};
        int inst = addChip(world, 0, 200, 100);
        int sA = addComp(world, SWITCH, 20, 60), sB = addComp(world, SWITCH, 20, 200);
        int lA = addComp(world, LAMP, 460, 60), lB = addComp(world, LAMP, 460, 200);
        addWire(world, sA, 0, inst, 0); addWire(world, sB, 0, inst, 1);
        addWire(world, inst, 0, lA, 0); addWire(world, inst, 1, lB, 0);
        int before = 0; for (auto& w : world.wires) if (w.alive) ++before;

        // 둘째 출력핀을 지운다 → 출력이 하나로 준다
        beginEdit(0);
        int last = -1;
        for (int i = 0; i < (int)world.comps.size(); ++i)
            if (world.comps[i].alive && world.comps[i].type == PIN_OUT) last = i;
        deleteComp(world, last);
        int cut = endEdit();
        settle();

        int after = 0; for (auto& w : world.wires) if (w.alive) ++after;
        bool onlyOne = (before - after == 1) && (cut == 1);
        // 남은 선은 멀쩡해야 한다
        setSwitch(sA, 1); settle();
        bool stillWorks = lit(world.comps[lA]) && !lit(world.comps[lB]);
        bool portsShrank = chips[0].tmpl.outPorts.size() == 1;

        std::snprintf(buf, sizeof(buf), "선 %d→%d개 (끊김 %d), 출력포트 %d개, 남은 선 멀쩡 %d",
                      before, after, cut, (int)chips[0].tmpl.outPorts.size(), (int)stillWorks);
        check("포트가 줄면 그 선만 끊긴다", onlyOne && portsShrank && stillWorks, buf);
    }

    // 42. 자기 안에 자기를 못 넣는다
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear(); editStack.clear();
        int a3 = addComp(world, PIN_IN, 100, 100), b3 = addComp(world, PIN_OUT, 300, 100);
        addWire(world, a3, 0, b3, 0);
        createChip({ a3, b3 }, "혼자칩");
        world = SubSim{};
        bool opened = beginEdit(0);
        bool blockedPlace = false;
        for (int i = 1; i < toolCount(); ++i)
            if (toolChip(i) == 0) blockedPlace = !toolEnabled(i);
        toolFind.clear();
        bool blockedReopen = !beginEdit(0);        // 열려 있는 걸 또 못 연다
        bool blockedDelete = !deleteChip(0);       // 고치는 중엔 못 지운다
        endEdit();
        std::snprintf(buf, sizeof(buf), "열림 %d, 목록에서 막힘 %d, 또 열기 막힘 %d, 지우기 막힘 %d",
                      (int)opened, (int)blockedPlace, (int)blockedReopen, (int)blockedDelete);
        check("자기 안에 자기를 못 넣는다",
              opened && blockedPlace && blockedReopen && blockedDelete, buf);
        editStack.clear();
    }

    // 44. 묶음·풀음이 가는 선과 굵은 선을 오간다
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear(); editStack.clear();
        int bu = addComp(world, BUNDLE, 200, 100);   // 기본 8비트
        int sp = addComp(world, SPLIT, 500, 100);
        bool w8 = (widthOf(world.comps[bu]) == 8) && (nIn(world.comps[bu]) == 8)
                  && (nOut(world.comps[sp]) == 8);
        // 굵은 선끼리 잇는다
        addWire(world, bu, 0, sp, 0);
        int joined = 0; for (auto& w : world.wires) if (w.alive) ++joined;

        // 스위치 여덟으로 0b10110101 = 181 을 만든다
        int want = 181;
        std::vector<int> sw;
        for (int i = 0; i < 8; ++i) {
            int q = addComp(world, SWITCH, 20, 40 + i * 40);
            sw.push_back(q);
            addWire(world, q, 0, bu, i);
            setSwitch(q, (want >> i) & 1);
        }
        // 풀음 뒤에 전구 여덟
        std::vector<int> lp;
        for (int i = 0; i < 8; ++i) {
            int q = addComp(world, LAMP, 700, 40 + i * 40);
            lp.push_back(q);
            addWire(world, sp, i, q, 0);
        }
        settle();
        int got = 0;
        for (int i = 0; i < 8; ++i) if (lit(world.comps[lp[i]])) got |= (1 << i);
        Val onWire = world.comps[bu].out.empty() ? 0 : world.comps[bu].out[0];

        std::snprintf(buf, sizeof(buf), "폭 8 %d, 굵은 선 %d개, 넣은 값 %d → 선에 %d → 나온 값 %d",
                      (int)w8, joined, want, (int)onWire, got);
        check("묶음·풀음이 값을 실어 나른다", w8 && joined == 1 && onWire == want && got == want, buf);
    }

    // 45. 폭이 다르면 안 이어진다
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear();
        int sw = addComp(world, SWITCH, 20, 20);          // 1비트
        int bu = addComp(world, BUNDLE, 200, 20);         // 8비트 출력
        int lp = addComp(world, LAMP, 400, 20);           // 1비트 입력
        addWire(world, bu, 0, lp, 0);                     // 8비트 → 1비트, 막혀야 한다
        int bad = 0; for (auto& w : world.wires) if (w.alive) ++bad;
        addWire(world, sw, 0, bu, 0);                     // 1비트 → 1비트, 되어야 한다
        int good = 0; for (auto& w : world.wires) if (w.alive) ++good;
        std::snprintf(buf, sizeof(buf), "폭 다른 것 이은 선 %d개(0이어야), 맞는 것 %d개", bad, good);
        check("폭이 다르면 안 이어진다", bad == 0 && good == 1, buf);
    }

    // 46. 클럭이 제 박자로 뒤집힌다
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear();
        int ck = addComp(world, CLOCK, 100, 100);
        world.comps[ck].aux = 2;                          // CLOCK_TICKS[2] = 4틱
        int per = CLOCK_TICKS[2];
        // 240틱 도는 동안 몇 번 뒤집히나 — 주기의 두 배마다 한 바퀴
        int flips = 0; bool last = lit(world.comps[ck]);
        for (int i = 0; i < 240; ++i) {
            tickSub(world);
            bool now = lit(world.comps[ck]);
            if (now != last) { ++flips; last = now; }
        }
        int wantFlips = 240 / per;
        bool ok = std::abs(flips - wantFlips) <= 1;
        // 빠르기를 바꾸면 뒤집는 횟수도 바뀐다
        world = SubSim{};
        int ck2 = addComp(world, CLOCK, 100, 100);
        world.comps[ck2].aux = 0;                         // 1틱마다
        int fast = 0; last = lit(world.comps[ck2]);
        for (int i = 0; i < 240; ++i) {
            tickSub(world);
            bool now = lit(world.comps[ck2]);
            if (now != last) { ++fast; last = now; }
        }
        std::snprintf(buf, sizeof(buf), "%d틱 주기로 240틱에 %d번(바라는 값 %d), 1틱 주기면 %d번",
                      per, flips, wantFlips, fast);
        check("클럭이 제 박자로 뒤집힌다", ok && fast > flips * 2, buf);
    }

    // 47. 램이 쓰고 읽는다 (클럭이 올라갈 때만 쓴다)
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear();
        int ram = addComp(world, RAM, 300, 100);
        bool ports = nIn(world.comps[ram]) == 4 && nOut(world.comps[ram]) == 1
                     && inWidth(world.comps[ram], 0) == RAM_ADDRW
                     && outWidth(world.comps[ram], 0) == RAM_DATAW;

        // 주소·데이터·쓰기·클럭을 손으로 넣어 준다
        auto poke = [&](int addr, int data, int we, int clk) {
            Comp& c = world.comps[ram];
            c.in.assign(4, 0);
            c.in[0] = (Val)addr; c.in[1] = (Val)data; c.in[2] = (Val)we; c.in[3] = (Val)clk;
            // tickSub 이 in 을 지우므로 램 계산만 흉내 낸다
            if (c.mem.size() != (size_t)RAM_WORDS) c.mem.assign(RAM_WORDS, 0);
            if (we && clk && !c.lastClk) c.mem[addr % RAM_WORDS] = maskTo((Val)data, RAM_DATAW);
            c.lastClk = clk ? 1 : 0;
            c.out.assign(1, c.mem[addr % RAM_WORDS]);
            return (int)c.out[0];
        };
        poke(7, 200, 1, 0);                 // 쓰기는 켰지만 클럭이 아직 0
        int beforeEdge = poke(7, 200, 1, 0);
        int atEdge     = poke(7, 200, 1, 1);   // 여기서 써진다
        poke(7, 99, 0, 0);                     // 쓰기 끄고
        int keptAfter  = poke(7, 99, 0, 1);    // 클럭이 올라가도 안 써져야
        int other      = poke(8, 0, 0, 0);     // 딴 자리는 0

        std::snprintf(buf, sizeof(buf), "포트 %d, 쓰기 전 %d, 클럭 올라갈 때 %d, 쓰기 끄면 %d, 딴 자리 %d",
                      (int)ports, beforeEdge, atEdge, keptAfter, other);
        check("램이 클럭에 맞춰 쓰고 늘 읽는다",
              ports && beforeEdge == 0 && atEdge == 200 && keptAfter == 200 && other == 0, buf);
    }

    // 48. 폭·클럭·램 속이 저장됐다 돌아온다
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear();
        int pin = addComp(world, PIN_IN, 50, 50);
        world.comps[pin].aux = 12; resizePorts(world.comps[pin]);      // 12비트 핀
        int ck  = addComp(world, CLOCK, 150, 50); world.comps[ck].aux = 4;
        int ram = addComp(world, RAM, 250, 50);
        world.comps[ram].mem.assign(RAM_WORDS, 0);
        world.comps[ram].mem[3] = 77; world.comps[ram].mem[200] = 255;
        bool wrote = saveState();

        world = SubSim{}; chips.clear();
        bool read = loadState();
        bool widthOK = read && (int)world.comps.size() > pin && widthOf(world.comps[pin]) == 12
                       && outWidth(world.comps[pin], 0) == 12;
        bool clkOK   = read && (int)world.comps.size() > ck && world.comps[ck].aux == 4;
        bool memOK   = read && (int)world.comps.size() > ram
                       && world.comps[ram].mem.size() == (size_t)RAM_WORDS
                       && world.comps[ram].mem[3] == 77 && world.comps[ram].mem[200] == 255;
        std::snprintf(buf, sizeof(buf), "폭 12 %d, 클럭 주기 %d, 램[3]=%d·램[200]=%d",
                      (int)widthOK, clkOK ? world.comps[ck].aux : -1,
                      memOK ? (int)world.comps[ram].mem[3] : -1,
                      memOK ? (int)world.comps[ram].mem[200] : -1);
        check("폭·클럭·램 속이 저장된다", wrote && read && widthOK && clkOK && memOK, buf);
        std::remove(savePath().c_str());
    }

    // 49. 클럭·묶음·램을 진짜로 이어서 돌려 본다 (CPU 의 뼈대)
    //     손으로 값을 꽂지 않고, 판에 회로를 짜서 시뮬레이션만으로 확인한다.
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear();
        int ram = addComp(world, RAM, 600, 200);
        int ck  = addComp(world, CLOCK, 100, 500);
        world.comps[ck].aux = 1;                       // 2틱마다 뒤집힘

        // 주소 8비트 = 스위치 여덟 → 묶음
        int abu = addComp(world, BUNDLE, 300, 100);
        std::vector<int> asw;
        for (int i = 0; i < 8; ++i) {
            int q = addComp(world, SWITCH, 100, 40 + i * 30);
            asw.push_back(q); addWire(world, q, 0, abu, i);
        }
        // 데이터 8비트
        int dbu = addComp(world, BUNDLE, 300, 320);
        std::vector<int> dsw;
        for (int i = 0; i < 8; ++i) {
            int q = addComp(world, SWITCH, 100, 300 + i * 30);
            dsw.push_back(q); addWire(world, q, 0, dbu, i);
        }
        int we = addComp(world, SWITCH, 300, 560);

        addWire(world, abu, 0, ram, 0);
        addWire(world, dbu, 0, ram, 1);
        addWire(world, we,  0, ram, 2);
        addWire(world, ck,  0, ram, 3);

        // 램이 내보내는 값을 풀어서 전구 여덟으로
        int osp = addComp(world, SPLIT, 800, 200);
        addWire(world, ram, 0, osp, 0);
        std::vector<int> lp;
        for (int i = 0; i < 8; ++i) {
            int q = addComp(world, LAMP, 950, 40 + i * 30);
            lp.push_back(q); addWire(world, osp, i, q, 0);
        }
        int wires = 0; for (auto& w : world.wires) if (w.alive) ++wires;

        auto setBits = [&](std::vector<int>& v, int val) {
            for (int i = 0; i < 8; ++i) setSwitch(v[i], (val >> i) & 1);
        };
        auto readLamps = [&] {
            int g = 0;
            for (int i = 0; i < 8; ++i) if (lit(world.comps[lp[i]])) g |= (1 << i);
            return g;
        };

        // 42번 자리에 137 을 쓴다 — 클럭이 알아서 올라갈 때까지 돌린다
        setBits(asw, 42); setBits(dsw, 137); setSwitch(we, 1);
        for (int i = 0; i < 40; ++i) tickSub(world);
        int wrote137 = (int)world.comps[ram].mem[42];

        // 쓰기 끄고 딴 값을 걸어 놔도 안 바뀌어야
        setSwitch(we, 0); setBits(dsw, 3);
        for (int i = 0; i < 40; ++i) tickSub(world);
        int kept = (int)world.comps[ram].mem[42];
        int shown = readLamps();

        // 딴 자리로 주소를 옮기면 0 이 보인다
        setBits(asw, 43);
        for (int i = 0; i < 10; ++i) tickSub(world);
        int elsewhere = readLamps();

        // 그 자리에 5 를 쓰고 돌아와도 42번은 그대로
        setBits(dsw, 5); setSwitch(we, 1);
        for (int i = 0; i < 40; ++i) tickSub(world);
        setSwitch(we, 0); setBits(asw, 42);
        for (int i = 0; i < 10; ++i) tickSub(world);
        int back = readLamps();

        std::snprintf(buf, sizeof(buf),
                      "선 %d개, 42번에 %d 씀, 쓰기 끄니 %d 유지(전구 %d), 43번은 %d, 돌아오니 %d",
                      wires, wrote137, kept, shown, elsewhere, back);
        check("클럭·묶음·램이 이어서 돈다",
              wrote137 == 137 && kept == 137 && shown == 137 && elsewhere == 0 && back == 137, buf);
    }

    // 50. 화면에 쓰는 글자가 전부 글꼴에 있다
    //     없는 기호를 쓰면 두부(□)가 나온다. ✎ 와 ▮ 로 두 번 당했다.
    {
        // 화면에 나오는 붙박이 글월 — 여기 없는 걸 새로 쓰면 이 검사에 안 걸리니
        // 기호를 쓸 일이 생기면 여기에도 넣어야 한다.
        static const char* USED[] = {
            "셈틀", "손 (고르기)", "게이트로 묶기", "전체 지우기", "학습", "샌드박스",
            "◀ 앞", "뒤 ▶", "힌트 (H)", "힌트 닫기", "채점하기  (Enter)", "×", "고",
            "나가기  (Enter)", "계속하기  (Esc)", "첫 화면으로 나갈까?",
            "고치는 중:  ", "Esc : 나가서 반영", "H 로 닫기", "저장됨", "멈춤 (P)",
            "이렇게 돌아야 한다", "차례대로 이렇게", "쓸 수 있는 것:", "핀 이름:", "커스텀 게이트 이름:",
            "Enter 로 만들기 · Esc 로 취소", "Enter 로 고치기 · Esc 로 취소",
            "고르면 바로 시작 · 안에서 Esc 로 여기 돌아옴 · [ ] 로 글자 크기",
            "배선과 게이트만으로 회로를 조합하는 판",
            "NAND 하나로 시작해서 계산기까지", "빈 판. 마음대로",
            "Ctrl+Z=되돌리기 · Ctrl+S=내보내기 · Esc=첫 화면",
            "Enter=채점 · Esc=첫 화면", "R=돌리기 · Ctrl+D=복사 · G=묶기",
            "핀 더블클릭=이름 · 가운데버튼=이동 · Ctrl+휠=확대",
            "Tab : 판 다시 보기", "입력핀", "출력핀", "학습 진도  ", "단계",
            "손 도구", "선 고름 — Del 로 지운다", "찾기 (/)", "찾는 게 없다",
            "줄 수는 화면만 바꾼다 (E)", "화면 ", "줄", "×",
            "접을 수 있는 건 묶음·풀음뿐 (3비트부터)", "개 접음", "개 펼침",
            "빈 곳=놓기 · 부품 잡고 끌면 옮기기 · 우클릭=손 도구 (선 위면 끊기)",
            "Shift+클릭=하나씩 · R=돌리기 · Q=접기 · F2=핀 이름 · Ctrl+C/V · G=묶기",
            "핀 하나만 골라야 이름을 고친다", "핀을 고르거나 핀 위에서 F2",
            "Enter=채점 · 끌어서 고르기(Shift 로 더하기) · 선은 우클릭으로 끊기",
            "끌어서 고르기(Shift 로 더하기) · Del=지우기 · 선은 우클릭으로 끊기",
            "넣음 · ", "뺌 · ", "개 고름",
            "복사할 걸 먼저 골라", "복사해 둔 게 없음", "복제할 걸 먼저 골라",
            "지울 걸 먼저 골라",
            "는 단계에서 받은 부품이라 못 지움", "군데서 쓰는 중이라 못 지움",
            "없어진 부품 개를 되살렸다",
            "학습", "단계 통과", "난이도", "부 · ", " / 30 · 목록",
            "누르면 그 단계로 · 통과한 데까지만 갈 수 있다 · Esc 로 첫 화면",
            "ON", "OFF", "·", "램", "비트", "틱", "→", "入", "出", "—", "¼", "½",
        };
        // 부품·단계 이름도 다 훑는다
        std::vector<std::string> all;
        for (auto* t : USED) all.push_back(t);
        for (int i = 0; i < TYPE_N; ++i) all.push_back(TYPES[i].name);
        for (int i = 0; i < PART_N; ++i) { all.push_back(PARTS[i].name); all.push_back(PARTS[i].what); }
        for (int i = 0; i < LESSON_N; ++i) {
            all.push_back(LESSONS[i].title);
            all.push_back(LESSONS[i].text);
            all.push_back(LESSONS[i].hint ? LESSONS[i].hint : "");
            all.push_back(LESSONS[i].allow ? LESSONS[i].allow : "");
            all.push_back(LESSONS[i].name ? LESSONS[i].name : "");
            for (int k = 0; k < LESSONS[i].nIn; ++k)  all.push_back(LESSONS[i].inName[k]);
            for (int k = 0; k < LESSONS[i].nOut; ++k) all.push_back(LESSONS[i].outName[k]);
        }

        int missing = 0; char first[64] = "";
        if (!fontOK) std::snprintf(first, sizeof(first), "글꼴을 못 읽음");
        else for (const auto& t : all) {
            const char* p2 = t.c_str();
            while (*p2) {
                uint32_t cp = nextCp(p2);
                if (cp == '\n' || cp == ' ' || cp < 32) continue;
                if (stbtt_FindGlyphIndex(&font, (int)cp) == 0) {
                    ++missing;
                    if (!first[0]) std::snprintf(first, sizeof(first), "U+%04X 가 글꼴에 없음", cp);
                }
            }
        }
        if (missing) std::snprintf(buf, sizeof(buf), "%s (모두 %d자)", first, missing);
        else         std::snprintf(buf, sizeof(buf), "글월 %d개 훑음, 빠진 글자 없음", (int)all.size());
        check("쓰는 글자가 다 글꼴에 있다", fontOK && missing == 0, buf);
    }

    // 51. 복사(Ctrl+C)는 판을 안 건드리고, 붙여넣기(Ctrl+V)가 꺼낸다
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear(); editStack.clear();
        clipboard.comps.clear(); clipboard.wires.clear();
        int a2 = addComp(world, SWITCH, 100, 100), g = addComp(world, T_AND, 250, 120);
        int b2 = addComp(world, SWITCH, 100, 220), l = addComp(world, LAMP, 400, 120);
        addWire(world, a2, 0, g, 0); addWire(world, b2, 0, g, 1); addWire(world, g, 0, l, 0);
        setSwitch(a2, 1); setSwitch(b2, 1); settle();
        int before = 0; for (auto& c : world.comps) if (c.alive) ++before;

        // 복사만 하면 판은 그대로여야 한다
        int copied = copyToClip({ a2, b2, g, l });
        int afterCopy = 0; for (auto& c : world.comps) if (c.alive) ++afterCopy;
        bool untouched = (afterCopy == before);

        // 붙여넣기는 그 자리에 꺼낸다
        std::vector<int> made = pasteClip(700, 500);
        settle();
        int afterPaste = 0; for (auto& c : world.comps) if (c.alive) ++afterPaste;
        bool grew = (afterPaste == before + 4) && ((int)made.size() == 4);
        bool moved = grew && world.comps[made[0]].x == 700 && world.comps[made[0]].y == 500;
        // 붙여넣은 것도 제대로 돈다 (안의 선까지 따라왔나)
        int newLamp = -1;
        for (int i : made) if (world.comps[i].type == LAMP) newLamp = i;
        bool works = (newLamp >= 0) && lit(world.comps[newLamp]);
        // 두 번 붙여넣어도 된다
        std::vector<int> made2 = pasteClip(900, 700);
        bool twice = ((int)made2.size() == 4);

        std::snprintf(buf, sizeof(buf), "복사 %d개(판 %d→%d 그대로 %d), 붙여넣기 %d개·자리 %d·돌아감 %d, 또 %d",
                      copied, before, afterCopy, (int)untouched,
                      (int)made.size(), (int)moved, (int)works, (int)twice);
        check("복사와 붙여넣기가 따로 논다",
              copied == 4 && untouched && grew && moved && works && twice, buf);
        clipboard.comps.clear(); clipboard.wires.clear();
    }

    // 52. 붙여넣기는 반만 고른 선을 안 가져온다
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear();
        clipboard.comps.clear(); clipboard.wires.clear();
        int a3 = addComp(world, SWITCH, 100, 100), l3 = addComp(world, LAMP, 400, 100);
        addWire(world, a3, 0, l3, 0);
        copyToClip({ a3 });                       // 스위치만 담는다
        bool noWire = clipboard.wires.empty();
        int before = 0; for (auto& w : world.wires) if (w.alive) ++before;
        pasteClip(100, 400);
        int after = 0; for (auto& w : world.wires) if (w.alive) ++after;
        std::snprintf(buf, sizeof(buf), "담긴 선 %d개(0이어야), 붙인 뒤 판의 선 %d→%d",
                      (int)clipboard.wires.size(), before, after);
        check("반만 고르면 선은 안 담긴다", noWire && before == after, buf);
        clipboard.comps.clear(); clipboard.wires.clear();
    }

    // 53. 목록 찾기가 걸러 준다
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear(); editStack.clear();
        toolFind.clear(); toolScroll = 0;
        // 이름이 뚜렷한 칩 몇 개 만들어 둔다
        const char* names[] = { "덧셈기8", "반가산기", "레지스터" };
        for (const char* n : names) {
            int p2 = addComp(world, PIN_IN, 100, 100), q2 = addComp(world, PIN_OUT, 300, 100);
            addWire(world, p2, 0, q2, 0);
            createChip({ p2, q2 }, n);
        }
        world = SubSim{};
        int all = shownCount();

        toolFind = "가산";                         // 덧셈기8 은 안 걸리고 반가산기만
        std::vector<int> v1 = shownTools();
        bool onlyAdder = ((int)v1.size() == 2) && v1[0] == 0 &&
                         std::string(toolName(v1[1])) == "반가산기";

        toolFind = "AND";                          // 붙박이 셋 (AND, NAND)
        std::vector<int> v2 = shownTools();
        int nAnd = 0;
        for (size_t k = 1; k < v2.size(); ++k) {
            std::string nm = toolName(v2[k]);
            if (nm.find("AND") == std::string::npos) nAnd = -999;
            else ++nAnd;
        }

        toolFind = "없는이름";                      // 손만 남아야
        int nNone = shownCount();
        bool onlyHand = (nNone == 1);

        toolFind.clear();                          // 지우면 다 돌아온다
        int nBack = shownCount();
        bool back = (nBack == all);

        std::snprintf(buf, sizeof(buf), "전부 %d개, '가산'→반가산기만 %d, 'AND'→%d개, 없는 이름→%d개(손만), 지우면 %d개",
                      all, (int)onlyAdder, nAnd, nNone, nBack);
        check("목록 찾기가 걸러 준다",
              onlyAdder && nAnd == 2 && onlyHand && back, buf);
        toolFind.clear();
    }

    // 54. 굴림 막대가 자리 안에 있고 끝까지 간다
    {
        int ow = WIN_W, oh = WIN_H; float ous = uiScale;
        bool ok = true; char detail[140] = "";
        for (float us : { 0.8f, 1.35f, 2.2f }) {
            uiScale = us; applyUiScale();
            WIN_W = WIN_W_MIN; WIN_H = WIN_H_MIN;
            screen = SC_SANDBOX; world = SubSim{}; chips.clear(); toolFind.clear();
            for (int i = 0; i < 8; ++i) {          // 넘치게 칩을 만든다
                int p2 = addComp(world, PIN_IN, 10, 10), q2 = addComp(world, PIN_OUT, 60, 10);
                addWire(world, p2, 0, q2, 0);
                char n[16]; std::snprintf(n, sizeof(n), "칩%d", i + 1);
                createChip({ p2, q2 }, n);
            }
            world = SubSim{};
            if (toolScrollMax() <= 0) { ok = false; std::snprintf(detail, sizeof(detail), "배율 %.1f: 넘치지가 않음", us); continue; }

            Rect t = scrollTrack();
            toolScroll = 0;              Rect th0 = scrollThumb();
            toolScroll = toolScrollMax(); Rect th1 = scrollThumb();
            if (th0.y < t.y || th1.y + th1.h > t.y + t.h + 1) {
                ok = false;
                std::snprintf(detail, sizeof(detail), "배율 %.1f: 막대가 길 밖으로 (%d..%d, 길 %d..%d)",
                              us, th0.y, th1.y + th1.h, t.y, t.y + t.h);
            }
            if (th1.y <= th0.y) {
                ok = false;
                std::snprintf(detail, sizeof(detail), "배율 %.1f: 굴려도 막대가 안 내려감", us);
            }
            toolScroll = 0;
        }
        uiScale = ous; applyUiScale(); WIN_W = ow; WIN_H = oh;
        world = SubSim{}; chips.clear();
        check("굴림 막대가 제자리에 있다", ok, ok ? "배율 셋 다 길 안에서 끝까지 감" : detail);
    }

    // 55. 휠로 굴리면 실제로 줄이 움직인다
    //     (예전에 굴리는 코드가 통째로 빠져 있었는데 아무도 못 잡았다)
    {
        int ow = WIN_W, oh = WIN_H; float ous = uiScale;
        uiScale = 1.35f; applyUiScale();
        WIN_W = WIN_W0; WIN_H = WIN_H0;
        screen = SC_SANDBOX; world = SubSim{}; chips.clear(); toolFind.clear(); toolScroll = 0;
        // 붙박이 14 + 손 1 만으로는 안 넘칠 수 있으니 칩을 넉넉히 만든다
        for (int i = 0; i < 12; ++i) {
            int p2 = addComp(world, PIN_IN, 10, 10), q2 = addComp(world, PIN_OUT, 60, 10);
            addWire(world, p2, 0, q2, 0);
            char n[16]; std::snprintf(n, sizeof(n), "칩%d", i + 1);
            createChip({ p2, q2 }, n);
        }
        world = SubSim{};

        bool overflows = toolScrollMax() > 0;
        int y0 = toolRect(0).y;
        // 휠 한 칸 = 아래로
        toolScroll -= -1 * S(40); clampToolScroll();
        int y1 = toolRect(0).y;
        // 위로 되돌리기
        toolScroll -= 1 * S(40); clampToolScroll();
        int y2 = toolRect(0).y;
        // 아무리 위로 굴려도 처음보다 위로는 안 간다
        for (int i = 0; i < 20; ++i) { toolScroll -= 1 * S(40); clampToolScroll(); }
        int y3 = toolRect(0).y;
        // 끝까지 내려도 마지막 줄이 자리 안
        for (int i = 0; i < 40; ++i) { toolScroll -= -1 * S(40); clampToolScroll(); }
        Rect last = toolRect(shownCount() - 1);
        bool endOK = (last.y + last.h <= toolBot() + S(2));

        uiScale = ous; applyUiScale(); WIN_W = ow; WIN_H = oh;
        world = SubSim{}; chips.clear(); toolScroll = 0;
        std::snprintf(buf, sizeof(buf), "넘침 %d, 첫 줄 y %d→%d→%d, 위 한계 %d, 끝까지 %d",
                      (int)overflows, y0, y1, y2, y3, (int)endOK);
        check("휠로 굴리면 줄이 움직인다",
              overflows && y1 < y0 && y2 == y0 && y3 == y0 && endOK, buf);
    }

    // 56. 학습에서 딴 칩을 샌드박스에서도 쓴다 (판은 여전히 따로)
    {
        std::remove(chipsPath().c_str());
        std::remove(savePathFor(SC_LEARN).c_str());
        std::remove(savePathFor(SC_SANDBOX).c_str());

        // 학습에서 칩 하나 만들고 저장
        screen = SC_LEARN; world = SubSim{}; chips.clear();
        int p2 = addComp(world, PIN_IN, 40, 40), q2 = addComp(world, PIN_OUT, 240, 40);
        addWire(world, p2, 0, q2, 0);
        createChip({ p2, q2 }, "학습칩");
        lessonAt = 5; lessonDone = 6;
        bool w1 = saveState();

        // 샌드박스로 가서 판을 따로 짜고 저장
        screen = SC_SANDBOX; world = SubSim{}; chips.clear();
        // 샌드박스 판 저장본은 아직 없다. 그래도 칩은 넘어와야 한다.
        bool r1 = loadState();
        (void)r1;
        bool chipCrossed = false;
        for (auto& c : chips) if (c.alive && c.name == "학습칩") chipCrossed = true;
        // 그 칩을 실제로 놓아 쓸 수 있어야 한다
        int cid = -1;
        for (int i = 0; i < (int)chips.size(); ++i) if (chips[i].alive) cid = i;
        int inst = (cid >= 0) ? addChip(world, cid, 100, 100) : -1;
        int sw = addComp(world, SWITCH, 20, 100), lp = addComp(world, LAMP, 300, 100);
        if (inst >= 0) { addWire(world, sw, 0, inst, 0); addWire(world, inst, 0, lp, 0); }
        setSwitch(sw, 1); settle();
        bool works = (inst >= 0) && lit(world.comps[lp]);
        addComp(world, T_NOR, 500, 300);                   // 샌드박스만의 판
        bool w2 = saveState();

        // 학습으로 돌아가면 판은 제 것, 칩은 그대로
        screen = SC_LEARN; world = SubSim{}; chips.clear();
        bool r2 = loadState();
        int learnComps = 0, learnNor = 0;
        for (auto& c : world.comps) if (c.alive) { ++learnComps; if (c.type == T_NOR) ++learnNor; }
        bool stillHasChip = false;
        for (auto& c : chips) if (c.alive && c.name == "학습칩") stillHasChip = true;
        // 칩을 묶으면 판에 상자 하나가 남는다. 샌드박스에서 놓은 NOR 은 안 넘어와야 한다.
        bool boardsSeparate = (learnComps == 1 && learnNor == 0);
        bool progKept = (lessonAt == 5 && lessonDone == 6);

        // 샌드박스 판은 제 것대로
        screen = SC_SANDBOX; world = SubSim{}; chips.clear();
        loadState();
        int sandComps = 0; for (auto& c : world.comps) if (c.alive) ++sandComps;

        std::snprintf(buf, sizeof(buf),
                      "판 저장본 없어도 칩 넘어감 %d·돌아감 %d·남음 %d, 판 %d/%d 따로 %d, 진도 %d, 다시 읽기 %d",
                      (int)chipCrossed, (int)works, (int)stillHasChip,
                      learnComps, sandComps,
                      (int)(boardsSeparate && sandComps > learnComps), (int)progKept, (int)r2);
        check("칩은 같이 쓰고 판은 따로다",
              w1 && chipCrossed && works && w2 && r2 && stillHasChip
              && boardsSeparate && sandComps > learnComps && progKept, buf);

        std::remove(chipsPath().c_str());
        std::remove(savePathFor(SC_LEARN).c_str());
        std::remove(savePathFor(SC_SANDBOX).c_str());
        screen = SC_SANDBOX; lessonAt = lessonDone = 0;
    }

    // 57. 접어도 이어 둔 선이 안 끊기고, 상자만 작아진다
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear(); editStack.clear();
        int bu = addComp(world, BUNDLE, 300, 100);      // 8비트
        int sp = addComp(world, SPLIT, 700, 100);
        addWire(world, bu, 0, sp, 0);
        std::vector<int> sw, lp;
        for (int i = 0; i < 8; ++i) {
            int q = addComp(world, SWITCH, 60, 40 + i * 40);
            sw.push_back(q); addWire(world, q, 0, bu, i);
            int l = addComp(world, LAMP, 900, 40 + i * 40);
            lp.push_back(l); addWire(world, sp, i, l, 0);
        }
        int want = 181;
        for (int i = 0; i < 8; ++i) setSwitch(sw[i], (want >> i) & 1);
        settle();
        int got0 = 0; for (int i = 0; i < 8; ++i) if (lit(world.comps[lp[i]])) got0 |= (1 << i);
        int wires0 = 0; for (auto& w : world.wires) if (w.alive) ++wires0;
        int h0 = compH(world.comps[bu]);
        int cx0 = world.comps[bu].x + compW(world.comps[bu])/2;
        int cy0 = world.comps[bu].y + compH(world.comps[bu])/2;

        // 접는다
        world.comps[bu].aux |= FOLD_BIT;
        { Comp& c = world.comps[bu];             // 가운데 맞추기 (doFold 와 같은 계산)
          c.x = cx0 - compW(c)/2; c.y = cy0 - compH(c)/2; }
        settle();
        int h1 = compH(world.comps[bu]);
        int wires1 = 0; for (auto& w : world.wires) if (w.alive) ++wires1;
        int got1 = 0; for (int i = 0; i < 8; ++i) if (lit(world.comps[lp[i]])) got1 |= (1 << i);
        bool ports = (nIn(world.comps[bu]) == 8);       // 포트 수는 그대로여야 한다
        int cx1 = world.comps[bu].x + compW(world.comps[bu])/2;
        int cy1 = world.comps[bu].y + compH(world.comps[bu])/2;
        bool centered = std::abs(cx1 - cx0) <= 1 && std::abs(cy1 - cy0) <= 1;

        // 저장했다 열어도 접힌 채로
        bool w = saveState();
        world = SubSim{}; chips.clear();
        bool r = loadState();
        bool keptFold = r && (int)world.comps.size() > bu && isFolded(world.comps[bu]);
        bool keptW = keptFold && widthOf(world.comps[bu]) == 8;
        settle();
        int got2 = 0;
        for (int i = 0; i < 8 && (int)world.comps.size() > lp[i]; ++i)
            if (lit(world.comps[lp[i]])) got2 |= (1 << i);

        // 2비트짜리는 접을 게 없다
        int small = addComp(world, BUNDLE, 10, 10);
        world.comps[small].aux = 2; resizePorts(world.comps[small]);
        bool noSmall = !canFold(world.comps[small]);

        std::snprintf(buf, sizeof(buf),
                      "높이 %d→%d, 선 %d→%d개, 값 %d→%d→%d, 포트 %d, 가운데 %d, 저장 %d·폭 %d, 2비트 막힘 %d",
                      h0, h1, wires0, wires1, got0, got1, got2, (int)ports, (int)centered,
                      (int)keptFold, (int)keptW, (int)noSmall);
        check("접어도 선이 안 끊긴다",
              h1 < h0 && wires1 == wires0 && got0 == want && got1 == want && got2 == want
              && ports && centered && w && keptFold && keptW && noSmall, buf);
        std::remove(savePath().c_str());
        std::remove(chipsPath().c_str());
    }

    // 58. 스위치묶음이 눌린 비트대로 값을 낸다
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear(); editStack.clear();
        int sb = addComp(world, SWBANK, 200, 100);
        bool w8 = widthOf(world.comps[sb]) == 8 && nOut(world.comps[sb]) == 1
                  && outWidth(world.comps[sb], 0) == 8 && nIn(world.comps[sb]) == 0;
        int sp = addComp(world, SPLIT, 500, 100);
        addWire(world, sb, 0, sp, 0);
        std::vector<int> lp;
        for (int i = 0; i < 8; ++i) {
            int q = addComp(world, LAMP, 700, 40 + i * 40);
            lp.push_back(q); addWire(world, sp, i, q, 0);
        }
        // 비트를 눌러 181 을 만든다 (누른 자리를 찾아 뒤집는 식)
        int want = 181;
        for (int i = 0; i < 8; ++i) {
            Comp& c = world.comps[sb];
            int w = compW(c), h = compH(c), wd = widthOf(c), pad = 4;
            int rowH = (h - 20 - pad) / wd;
            int yy = (c.y + h - 20) - i * rowH - rowH / 2;    // 그 칸 가운데
            int bit = swBankBitAt(c, c.x + w / 2, yy);
            if (bit == i && ((want >> i) & 1))
                c.out[0] = maskTo((Val)(c.out[0] ^ (1u << bit)), wd);
        }
        settle();
        int got = 0; for (int i = 0; i < 8; ++i) if (lit(world.comps[lp[i]])) got |= (1 << i);
        Val onWire = world.comps[sb].out.empty() ? 0 : world.comps[sb].out[0];

        // 저장했다 열어도 눌린 그대로
        bool wr = saveState();
        world = SubSim{}; chips.clear();
        bool rd = loadState();
        Val after = (rd && (int)world.comps.size() > sb) ? world.comps[sb].out[0] : 0;

        std::snprintf(buf, sizeof(buf), "폭 8·출력만 %d, 눌러서 %d, 전구 %d, 저장 뒤 %d",
                      (int)w8, (int)onWire, got, (int)after);
        check("스위치묶음이 눌린 대로 값을 낸다",
              w8 && onWire == want && got == want && wr && rd && after == want, buf);
        std::remove(savePath().c_str()); std::remove(chipsPath().c_str());
    }

    // 59. 화면이 줄마다 값을 받는다
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear();
        int sc2 = addComp(world, SCREEN, 300, 100);
        bool def = (screenRows(world.comps[sc2]) == 8) && (widthOf(world.comps[sc2]) == 8)
                   && (nIn(world.comps[sc2]) == 8) && (nOut(world.comps[sc2]) == 0)
                   && inWidth(world.comps[sc2], 0) == 8;

        // 줄 수를 5로 줄이면 포트도 5개
        world.comps[sc2].aux = (world.comps[sc2].aux & ~(0x1F << SCR_ROWS_SHIFT))
                             | (5 << SCR_ROWS_SHIFT);
        resizePorts(world.comps[sc2]);
        bool five = (nIn(world.comps[sc2]) == 5);
        // 칸 수를 4로
        world.comps[sc2].aux = (world.comps[sc2].aux & ~0xFF) | 4;
        resizePorts(world.comps[sc2]);
        bool four = (inWidth(world.comps[sc2], 0) == 4);
        int wSz = compW(world.comps[sc2]), hSz = compH(world.comps[sc2]);

        // 값을 넣어 본다
        std::vector<int> sb;
        for (int r = 0; r < 5; ++r) {
            int q = addComp(world, SWBANK, 60, 40 + r * 90);
            world.comps[q].aux = 4; resizePorts(world.comps[q]);
            world.comps[q].out.assign(1, (Val)(r * 3));
            sb.push_back(q);
            addWire(world, q, 0, sc2, r);
        }
        settle();
        bool got = true;
        for (int r = 0; r < 5; ++r)
            if (world.comps[sc2].in[r] != (Val)(r * 3)) got = false;

        // 16×16 까지 커진다
        world.comps[sc2].aux = 16 | (16 << SCR_ROWS_SHIFT);
        resizePorts(world.comps[sc2]);
        bool big = (nIn(world.comps[sc2]) == 16) && (inWidth(world.comps[sc2], 0) == 16)
                   && compW(world.comps[sc2]) > wSz && compH(world.comps[sc2]) > hSz;

        std::snprintf(buf, sizeof(buf), "기본 8×8 %d, 5줄 %d·4칸 %d, 줄마다 값 %d, 16×16 %d",
                      (int)def, (int)five, (int)four, (int)got, (int)big);
        check("화면이 줄마다 값을 받는다", def && five && four && got && big, buf);
    }

    // 60. 폭 있는 입력핀은 칸을 눌러 비트를 넣는다 (스위치묶음과 같게)
    {
        screen = SC_SANDBOX; world = SubSim{}; chips.clear(); editStack.clear();
        int p1 = addComp(world, PIN_IN, 100, 100);          // 1비트 — 예전처럼
        int p8 = addComp(world, PIN_IN, 400, 100);
        world.comps[p8].aux = 8; resizePorts(world.comps[p8]);

        bool oneBitPlain = !isBitBank(world.comps[p1]);
        bool eightIsBank = isBitBank(world.comps[p8]);
        bool taller = compH(world.comps[p8]) > compH(world.comps[p1]);

        // 칸을 눌러 181 을 만든다
        int want = 181;
        for (int i = 0; i < 8; ++i) {
            Comp& c = world.comps[p8];
            int w = compW(c), h = compH(c), wd = widthOf(c), pad = 4, top = 18;
            int rowH = (h - 20 - pad - top) / wd;
            int yy = (c.y + h - 20) - i * rowH - rowH / 2;
            int bit = swBankBitAt(c, c.x + w / 2, yy);
            if (bit == i && ((want >> i) & 1))
                c.out[0] = maskTo((Val)(c.out[0] ^ (1u << bit)), wd);
        }
        // 그 값이 선을 타고 나가는지
        int sp = addComp(world, SPLIT, 700, 100);
        addWire(world, p8, 0, sp, 0);
        std::vector<int> lp;
        for (int i = 0; i < 8; ++i) {
            int q = addComp(world, LAMP, 900, 40 + i * 40);
            lp.push_back(q); addWire(world, sp, i, q, 0);
        }
        settle();
        int got = 0; for (int i = 0; i < 8; ++i) if (lit(world.comps[lp[i]])) got |= (1 << i);

        // 1비트 핀은 여전히 그냥 뒤집힌다
        world.comps[p1].out[0] = !world.comps[p1].out[0];
        bool plainWorks = (world.comps[p1].out[0] == 1);

        std::snprintf(buf, sizeof(buf), "1비트는 그대로 %d, 8비트는 칸 %d·더 큼 %d, 눌러서 %d, 1비트 토글 %d",
                      (int)oneBitPlain, (int)eightIsBank, (int)taller, got, (int)plainWorks);
        check("폭 있는 입력핀은 칸으로 넣는다",
              oneBitPlain && eightIsBank && taller && got == want && plainWorks, buf);
    }

    // 61. 상품 칩을 지워도 되살아난다 (지우고 저장하면 못 깨던 상황)
    {
        screen = SC_LEARN; world = SubSim{}; chips.clear(); editStack.clear();
        // 플립플롭까지 실제로 풀어서 칩을 모은다.
        // 플립플롭은 아직 아무 칩도 안 쓰고 있어서 지울 수 있다 —
        // 실제로 막혔던 상황이 이거다.
        int upto = lessonByName("플립플롭") + 1;
        char why[200] = "";
        for (int i = 0; i < upto; ++i) {
            lessonAt = i; setupLesson(i);
            if (!buildSolution(i, why, sizeof(why))) { std::snprintf(buf, sizeof(buf), "%s", why); break; }
            bankLesson(LESSONS[i]);
        }
        lessonDone = upto;
        int before = 0; for (auto& c : chips) if (c.alive) ++before;

        // 방금 받은 플립플롭을 지운다 — 다음 단계(레지스터)가 이걸 쓴다
        int victim = -1;
        for (int i = 0; i < (int)chips.size(); ++i)
            if (chips[i].alive && chips[i].name == "플립플롭") victim = i;
        world = SubSim{};                        // 판을 비워 쓰는 데가 없게
        bool wiped = (victim >= 0) && deleteChip(victim);
        int afterDel = 0; for (auto& c : chips) if (c.alive) ++afterDel;

        // 이 상태로는 다음 단계(레지스터)를 못 푼다
        int next = lessonByName("레지스터");
        lessonAt = next; setupLesson(next);
        char err[200] = "";
        bool stuck = !buildSolution(next, err, sizeof(err));

        // 되살리기
        int fixed = repairLessonChips();
        int afterFix = 0; for (auto& c : chips) if (c.alive) ++afterFix;
        bool backAgain = false;
        for (auto& c : chips) if (c.alive && c.name == "AND짜기") backAgain = true;

        // 이제 다시 풀린다
        lessonAt = next; setupLesson(next);
        bool solves = buildSolution(next, err, sizeof(err))
                   && gradeLesson(LESSONS[next], err, sizeof(err));

        std::snprintf(buf, sizeof(buf), "칩 %d개 → 지우니 %d개(막힘 %d) → 되살리니 %d개, 돌아옴 %d, 다시 풀림 %d",
                      before, afterDel, (int)stuck, afterFix, (int)backAgain, (int)solves);
        check("지운 상품 칩이 되살아난다",
              wiped && stuck && fixed == 1 && backAgain && solves && afterFix == before, buf);
        screen = SC_SANDBOX; lessonAt = lessonDone = 0; chips.clear(); world = SubSim{};
    }

    // 62. 부가 단계를 빠짐없이 나눠 가진다
    {
        bool ok = (PARTS[0].first == 0);
        char why[160] = "";
        for (int p = 1; p < PART_N && ok; ++p)
            if (PARTS[p].first <= PARTS[p-1].first) {
                ok = false; std::snprintf(why, sizeof(why), "%d부 시작이 뒤로 안 간다", p+1);
            }
        for (int p = 0; p < PART_N && ok; ++p) {
            if (partEnd(p) <= PARTS[p].first) {
                ok = false; std::snprintf(why, sizeof(why), "%d부가 비었다", p+1);
            }
            if (PARTS[p].diff < 1 || PARTS[p].diff > DIFF_MAX) {
                ok = false; std::snprintf(why, sizeof(why), "%d부 난이도가 범위 밖", p+1);
            }
        }
        // 단계마다 부가 딱 하나여야 한다
        for (int i = 0; i < LESSON_N && ok; ++i) {
            int p = partOf(i);
            if (i < PARTS[p].first || i >= partEnd(p)) {
                ok = false; std::snprintf(why, sizeof(why), "%d단계가 엉뚱한 부", i+1);
            }
        }
        if (ok && partEnd(PART_N - 1) != LESSON_N) {
            ok = false; std::snprintf(why, sizeof(why), "마지막 부가 %d단계에서 끝난다", partEnd(PART_N-1));
        }
        if (ok) std::snprintf(buf, sizeof(buf), "%d부가 %d단계를 나눠 가짐 (난이도 %d~%d)",
                              PART_N, LESSON_N, PARTS[0].diff, PARTS[PART_N-1].diff);
        else    std::snprintf(buf, sizeof(buf), "%s", why);
        check("부가 단계를 빠짐없이 나눈다", ok, buf);
    }

    // 63. 단계 목록에서 누른 자리와 그린 자리가 같다
    {
        int ow = WIN_W, oh = WIN_H; float os = uiScale;
        bool ok = true; char why[200] = "";
        const int SZ[3][2] = { { WIN_W_MIN, WIN_H_MIN }, { 1400, 880 }, { 1920, 1200 } };
        int seen = 0;
        for (float sc : { 0.8f, 1.0f, 2.2f }) {
            uiScale = sc; applyUiScale();
            for (auto& z : SZ) {
                WIN_W = z[0]; WIN_H = z[1];
                pickScroll = 0;
                // 목록을 굴려 가며 모든 단계가 한 번은 제자리에서 잡히는지 본다
                for (int at = 0; at < LESSON_N && ok; ++at) {
                    pickScrollTo(at);
                    int y = pickTop() + pickRowY(at) - pickScroll;
                    if (y < pickTop() || y + pickLesH() > pickBot()) continue;   // 안 보이는 건 넘긴다
                    // 줄의 위·가운데·아래를 다 눌러 본다 (한 점만 보면 어긋나도 지나간다)
                    for (int dy : { 1, pickLesH()/2, pickLesH() - 1 }) {
                        int got = pickHit(pickColX() + S(60), y + dy);
                        ++seen;
                        if (got != at) {
                            ok = false;
                            std::snprintf(why, sizeof(why),
                                          "배율 %.1f %dx%d 에서 %d단계 줄의 %d번째 점을 누르니 %d 이 잡힘",
                                          sc, z[0], z[1], at + 1, dy, got + 1);
                            break;
                        }
                    }
                }
                // 목록 밖은 안 잡혀야 한다
                if (ok && pickHit(pickColX() - S(20), pickTop() + S(4)) != -1) {
                    ok = false; std::snprintf(why, sizeof(why), "칸 왼쪽 바깥이 잡힌다");
                }
                if (ok && pickHit(pickColX() + S(60), pickBot() + S(6)) != -1) {
                    ok = false; std::snprintf(why, sizeof(why), "도움말 자리가 잡힌다");
                }
            }
        }
        uiScale = os; applyUiScale(); WIN_W = ow; WIN_H = oh; pickScroll = 0;
        if (ok) std::snprintf(buf, sizeof(buf), "배율 셋 × 크기 셋에서 %d 자리 다 맞음", seen);
        else    std::snprintf(buf, sizeof(buf), "%s", why);
        check("단계 목록의 누른 자리가 맞는다", ok, buf);
    }

    std::printf("\n%s\n", failed ? "실패 있음" : "전부 통과");
    world = SubSim{}; chips.clear(); editStack.clear();
    return failed ? 1 : 0;
}



// 화면을 파일로 뽑는다 (문서에 넣을 그림용). 창을 안 띄우고도 된다.
// 그 자리에 배경 아닌 것이 그려졌나 (판이 통째로 사라지는 걸 잡으려고)
static bool anyPaint(SDL_Renderer* ren, int x, int y, int w, int h, uint32_t bg) {
    if (w <= 0 || h <= 0) return false;
    std::vector<uint8_t> px((size_t)w * h * 4);
    SDL_Rect r{ x, y, w, h };
    if (SDL_RenderReadPixels(ren, &r, SDL_PIXELFORMAT_ARGB8888, px.data(), w * 4) != 0)
        return true;                       // 못 읽으면 넘어간다
    for (int i = 0; i < w * h; ++i) {
        uint32_t c = ((uint32_t)px[i*4+2] << 16) | ((uint32_t)px[i*4+1] << 8) | px[i*4+0];
        if (c != bg) return true;
    }
    return false;
}

// 그 자리에 이 빛깔이 한 점이라도 있나 (표시가 진짜 그려졌는지 보려고)
static bool hasColor(SDL_Renderer* ren, int x, int y, int w, int h, uint32_t want) {
    if (w <= 0 || h <= 0) return false;
    std::vector<uint8_t> px((size_t)w * h * 4);
    SDL_Rect r{ x, y, w, h };
    if (SDL_RenderReadPixels(ren, &r, SDL_PIXELFORMAT_ARGB8888, px.data(), w * 4) != 0)
        return true;                       // 못 읽으면 넘어간다
    for (int i = 0; i < w * h; ++i) {
        uint32_t c = ((uint32_t)px[i*4+2] << 16) | ((uint32_t)px[i*4+1] << 8) | px[i*4+0];
        if (c == want) return true;
    }
    return false;
}

static void writePPM(SDL_Renderer* ren, const char* path) {
    int w = 0, h = 0;
    SDL_GetRendererOutputSize(ren, &w, &h);
    std::vector<uint8_t> px((size_t)w * h * 4);
    if (SDL_RenderReadPixels(ren, nullptr, SDL_PIXELFORMAT_ARGB8888,
                             px.data(), w * 4) != 0) {
        std::fprintf(stderr, "화면을 못 읽음: %s\n", SDL_GetError());
        return;
    }
    std::FILE* f = std::fopen(path, "wb");
    if (!f) { std::fprintf(stderr, "%s 못 씀\n", path); return; }
    std::fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (int i = 0; i < w * h; ++i) {
        uint8_t rgb[3] = { px[i*4+2], px[i*4+1], px[i*4+0] };   // ARGB → RGB
        std::fwrite(rgb, 1, 3, f);
    }
    std::fclose(f);
    std::printf("찍음: %s (%dx%d)\n", path, w, h);
}

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--test") == 0) return runTests();
    const bool shotMode = (argc > 1 && std::strcmp(argv[1], "--shot") == 0);
    const bool ansMode  = (argc > 1 && std::strcmp(argv[1], "--answers") == 0);

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::fprintf(stderr, "SDL 못 켬: %s\n", SDL_GetError());
        return 1;
    }
    applyUiScale();
    // 화면보다 큰 창은 안 띄운다 (배율을 키우면 처음 크기가 커진다)
    {
        SDL_DisplayMode dm;
        if (SDL_GetDesktopDisplayMode(0, &dm) == 0) {
            WIN_W0 = std::min(WIN_W0, dm.w - 80);
            WIN_H0 = std::min(WIN_H0, dm.h - 120);
        }
        WIN_W0 = std::max(WIN_W0, WIN_W_MIN);
        WIN_H0 = std::max(WIN_H0, WIN_H_MIN);
    }
    if (shotMode || ansMode) { WIN_W0 = 1400; WIN_H0 = 880; }
    SDL_Window* win = SDL_CreateWindow("셈틀",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIN_W0, WIN_H0,
        SDL_WINDOW_RESIZABLE);
    SDL_SetWindowMinimumSize(win, WIN_W_MIN, WIN_H_MIN);
    SDL_GetWindowSize(win, &WIN_W, &WIN_H);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    loadFont();

    // 진도만 먼저 읽어 둔다 — 첫 화면에 "몇 단계까지 했나" 를 보여 줘야 해서.
    // 판까지 읽히지만, 모드에 들어갈 때 그 모드 것으로 다시 읽는다.
    screen = SC_LEARN;
    if (!loadFrom(savePathFor(SC_LEARN))) { lessonAt = lessonDone = 0; }
    int learnAt = lessonAt, learnDone = lessonDone;
    screen = SC_MENU;
    world = SubSim{}; chips.clear();
    lessonAt = learnAt; lessonDone = learnDone;

    // 그림 뽑기: 장면을 하나씩 차리고 한 프레임 그린 뒤 파일로 저장한다.
    int shotAt = shotMode ? 0 : -1;
    int shotWait = 0;
    // 정답 그림 뽑기: 단계마다 정답 회로를 짓고 한 장씩 찍는다
    // 첫 화면 갈래가 continue 해 버리니 처음부터 학습 화면으로 둔다
    int ansAt = ansMode ? 0 : -1;
    int ansWait = 0;
    if (ansMode) { screen = SC_LEARN; chips.clear(); world = SubSim{}; }
    bool smokeBad = false;      // 스모크가 잡아낸 잘못

    // 화면 없이 그리기 경로가 안 죽는지 보는 모드. 네 방향·여러 배율을 다 훑는다.
    const bool smoke = env2("SEMTLE_SMOKE", "LOGIC_SMOKE") != nullptr;
    int smokeLeft = 0;
    if (smoke) {
        screen = SC_SANDBOX;
        for (int t = 0; t < TYPE_N; ++t) {
            int i = addComp(t, 40 + t * 120, 500);
            world.comps[i].rot = t & 3;                  // 네 방향 다
        }
        for (int t = 0; t + 1 < TYPE_N; ++t)
            addWire(world, t, 0, t + 1, 0);              // 돌아간 것끼리 선도 이어 본다
        {   // 고치기 화면도 밟아 보게 칩 하나 만들어 둔다
            int a2 = addComp(world, PIN_IN, 60, 60), b2 = addComp(world, PIN_OUT, 260, 60);
            addWire(world, a2, 0, b2, 0);
            createChip({ a2, b2 }, "스모크칩");
        }
        smokeLeft = 30;
    }

    int  tool = 0;
    bool running = true, paused = false;
    int  frame = 0;

    bool draggingWire = false; int wireFromComp = -1, wireFromPort = 0;
    bool draggingComp = false; int dragComp = -1, dragDX = 0, dragDY = 0; bool dragMoved = false;
    bool selecting = false; int selX0 = 0, selY0 = 0, selX1 = 0, selY1 = 0;   // 판 좌표
    bool selAdd = false;               // Shift 로 끌면 이미 고른 것에 더한다
    std::vector<int> sel;
    int selWire = -1;                  // 골라 둔 선 (Del 로 지운다)
    // 여럿 고른 걸 통째로 끌 때, 잡은 것에서 나머지가 얼마나 떨어져 있었나
    struct Follow { int i, dx, dy; };
    std::vector<Follow> dragBase;

    bool panning = false; int panSX = 0, panSY = 0; float panVX = 0, panVY = 0;

    // 글자 입력창. 칩 이름 짓기와 핀 이름 고치기가 같은 창을 쓴다.
    bool naming = false; std::string nameBuf;
    int  renameComp = -1;              // -1 이면 칩 만들기, 아니면 그 부품 이름 고치기
    int  mx = 0, my = 0;               // 화면 좌표
    int  wx = 0, wy = 0;               // 판 좌표 (히트 테스트는 다 이걸 쓴다)
    int  hoverTool = -1;
    bool dragScroll = false; int scrollGrabY = 0;   // 굴림 막대를 잡고 있나
    // 커서가 올라간 포트 (말풍선으로 이름을 보여 준다)
    int  tipComp = -1, tipPort = 0; bool tipIsIn = false;
    char toast[128] = ""; int toastLeft = 0;

    // 바뀐 게 있으면 표시해 뒀다가 잠깐 잠잠할 때 조용히 저장한다.
    // 매번 저장하면 선 하나 끄는 동안 파일을 수십 번 쓴다.
    bool dirty = false; int dirtyAt = 0;
    int savedFlash = 0;

    // 되돌리기. prev 는 늘 "마지막으로 바뀌고 난 뒤" 상태를 들고 있어서,
    // 다음에 뭔가 바뀔 때 그것을 되돌리기 더미에 밀어 넣으면 된다.
    std::vector<Snapshot> undoStack, redoStack;
    Snapshot prev = takeSnap();

    // 스위치를 껐다 켜는 건 회로를 "돌려 보는" 것이지 "고치는" 게 아니다.
    // 그것까지 되돌리기에 쌓으면 몇 번 만져 보는 사이 진짜 고친 기록이 밀려 나간다.
    // 그래서 저장만 하고 되돌리기에는 안 쌓는다.
    auto touchRun = [&] { dirty = true; dirtyAt = frame; };
    auto touch = [&] {
        touchRun();
        undoStack.push_back(std::move(prev));
        if ((int)undoStack.size() > UNDO_MAX) undoStack.erase(undoStack.begin());
        redoStack.clear();                       // 새로 손대면 앞길은 지운다
        prev = takeSnap();
    };

    auto syncW = [&] { wx = (int)s2wX(mx); wy = (int)s2wY(my); };

    auto say = [&](const char* s) { std::snprintf(toast, sizeof(toast), "%s", s); toastLeft = 120; };

    // 고른 것·끌던 것은 번호로 가리키는데 되돌리면 그 번호가 딴 걸 가리킬 수 있다
    auto dropHandles = [&] {
        sel.clear(); selWire = -1; dragBase.clear();
        draggingWire = draggingComp = selecting = false;
        wireFromComp = dragComp = tipComp = -1;
    };
    auto doUndo = [&] {
        if (undoStack.empty()) { say("되돌릴 게 없음"); return; }
        redoStack.push_back(takeSnap());
        putSnap(undoStack.back()); undoStack.pop_back();
        prev = takeSnap();
        dropHandles();
        dirty = true; dirtyAt = frame;
        char b[48]; std::snprintf(b, sizeof(b), "되돌림 (%d단계 남음)", (int)undoStack.size());
        say(b);
    };
    auto doRedo = [&] {
        if (redoStack.empty()) { say("다시 할 게 없음"); return; }
        undoStack.push_back(takeSnap());
        putSnap(redoStack.back()); redoStack.pop_back();
        prev = takeSnap();
        dropHandles();
        dirty = true; dirtyAt = frame;
        say("다시 함");
    };
    // ── 모드 드나들기 ──
    char lesMsg[160] = ""; bool lesOK = false; int lesLeft = 0;
    bool confirming = false;                       // 나갈까 묻는 중

    auto enterMode = [&](int sc) {
        screen = sc;
        dropHandles(); tool = 0; paused = false;
        undoStack.clear(); redoStack.clear();
        lesLeft = 0; lesMsg[0] = 0; hintOn = false;
        if (!loadFrom(savePathFor(sc))) {          // 저장본이 없으면 처음부터
            world = SubSim{}; chips.clear();
            if (sc == SC_SANDBOX) { seedDemo(); viewZoom = 1; viewX = viewY = 0; }
            else                  { lessonAt = lessonDone = 0; setupLesson(0); }
        } else if (sc == SC_LEARN && world.comps.empty()) {
            setupLesson(lessonAt);
        }
        if (sc == SC_LEARN) {
            int fixed = repairLessonChips();      // 지워 버린 상품 칩을 되살린다
            if (fixed) {
                char b2[64];
                std::snprintf(b2, sizeof(b2), "없어진 부품 %d개를 되살렸다", fixed);
                say(b2);
                dirty = true; dirtyAt = frame;
            }
        }
        prev = takeSnap();
        dirty = false;
    };
    // 학습을 고르면 단계 목록부터 — 저장본을 읽어야 진도를 안다
    auto openPick = [&] {
        if (screen != SC_LEARN) enterMode(SC_LEARN);
        pickScrollTo(lessonAt);
        screen = SC_PICK;
    };
    auto leaveToMenu = [&] {
        while (!editStack.empty()) endEdit();      // 고치던 칩은 마무리하고 나간다
        if (screen != SC_MENU && shotAt < 0 && ansAt < 0) saveState();   // 나가기 전에 써 둔다
        screen = SC_MENU;
        confirming = false;
        dropHandles(); dirty = false;
    };

    // ── 채점 ──
    auto doGrade = [&] {
        const Lesson& L = LESSONS[std::clamp(lessonAt, 0, LESSON_N - 1)];
        Snapshot keep = takeSnap();                // 채점하며 스위치를 흔드니 되돌려 놓는다
        char why[160] = "";
        bool ok = gradeLesson(L, why, sizeof(why));
        putSnap(keep);
        std::snprintf(lesMsg, sizeof(lesMsg), "%s",
                      ok ? (L.bank ? "통과! 이제 부품으로 쓸 수 있다" : "통과!") : why);
        lesOK = ok; lesLeft = 400;
        if (ok) {
            bankLesson(L);
            if (lessonDone < lessonAt + 1) lessonDone = lessonAt + 1;
            dirty = true; dirtyAt = frame;
        }
    };
    auto gotoLesson = [&](int at) {
        if (at < 0 || at >= LESSON_N || at > lessonDone) return;
        saveState();
        lessonAt = at;
        setupLesson(at);
        {   // 앞 단계 상품 칩이 없으면 여기서도 되살린다
            int fixed = repairLessonChips();
            if (fixed) {
                char b2[64];
                std::snprintf(b2, sizeof(b2), "없어진 부품 %d개를 되살렸다", fixed);
                say(b2);
            }
        }
        dropHandles(); tool = 0;
        undoStack.clear(); redoStack.clear();
        prev = takeSnap();
        lesLeft = 0; lesMsg[0] = 0; hintOn = false;    // 단계가 바뀌면 힌트도 접는다
        dirty = true; dirtyAt = frame;
    };

    // 판에 놓인 걸 다 보이게 맞춘다
    auto fitView = [&]() {
        int x0 = 1<<30, y0 = 1<<30, x1 = -(1<<30), y1 = -(1<<30), n = 0;
        for (const auto& c : world.comps) {
            if (!c.alive) continue;
            x0 = std::min(x0, c.x); y0 = std::min(y0, c.y);
            x1 = std::max(x1, c.x + compW(c)); y1 = std::max(y1, c.y + compH(c));
            ++n;
        }
        if (!n) { viewZoom = 1.0f; viewX = viewY = 0; return; }
        int cw = canvasR() - canvasL();
        float zx = (float)(cw - 80) / std::max(1, x1 - x0);
        float zy = (float)(WIN_H - 80) / std::max(1, y1 - y0);
        viewZoom = std::clamp(std::min(zx, zy), ZOOM_MIN, ZOOM_MAX);
        viewX = x0 - (cw / viewZoom - (x1 - x0)) / 2;
        viewY = y0 - (WIN_H / viewZoom - (y1 - y0)) / 2;
    };

    // ── 칩 고치기 ──
    auto doEditChip = [&](int cid) {
        if (cid < 0 || cid >= (int)chips.size() || !chips[cid].alive) return;
        if (editingChip(cid)) { say("이미 고치는 중"); return; }
        if (!beginEdit(cid)) { say("그 칩은 못 엶"); return; }
        dropHandles(); tool = 0;
        undoStack.clear(); redoStack.clear(); prev = takeSnap();
        char b2[96]; std::snprintf(b2, sizeof(b2), "'%s' 속으로 들어옴 · Esc 로 나가기",
                                   chips[cid].name.c_str());
        say(b2);
        fitView();
    };
    auto doEndEdit = [&] {
        if (editStack.empty()) return;
        std::string nm = chips[editStack.back().chipId].name;
        int cut = endEdit();
        dropHandles(); tool = 0;
        undoStack.clear(); redoStack.clear(); prev = takeSnap();
        char b2[128];
        if (cut > 0) std::snprintf(b2, sizeof(b2), "'%s' 고침 — 없어진 포트에 걸린 선 %d개 끊김",
                                   nm.c_str(), cut);
        else         std::snprintf(b2, sizeof(b2), "'%s' 고침", nm.c_str());
        say(b2);
        dirty = true; dirtyAt = frame;
    };

    auto doExport = [&] {
        std::string path;
        if (!fileDialog(true, path)) { say("파일 창을 못 띄움 (zenity 없음?)"); return; }
        say(saveTo(path, true) ? "내보냄" : "내보내기 실패");
    };
    auto doImport = [&] {
        std::string path;
        if (!fileDialog(false, path)) { say("파일 창을 못 띄움 (zenity 없음?)"); return; }
        Snapshot before = takeSnap();
        if (loadFrom(path)) {
            undoStack.push_back(std::move(before));      // 가져오기도 되돌릴 수 있게
            if ((int)undoStack.size() > UNDO_MAX) undoStack.erase(undoStack.begin());
            redoStack.clear();
            prev = takeSnap();
            dropHandles();
            dirty = true; dirtyAt = frame;
            say("가져옴");
        } else {
            say("파일을 못 읽음");                        // loadFrom 이 알아서 되돌려 놓는다
        }
    };

    auto startNaming = [&]() {
        if (sel.empty()) return;
        naming = true; renameComp = -1; nameBuf.clear(); SDL_StartTextInput();
    };
    // 핀 이름 고치기 (더블클릭)
    auto startRename = [&](int ci) {
        naming = true; renameComp = ci;
        nameBuf = world.comps[ci].label;
        SDL_StartTextInput();
    };
    // 고른 것 복사. 새로 만든 것이 고른 상태가 된다.
    // Ctrl+C — 담기만 한다. 판은 안 바뀐다.
    auto doCopy = [&]() {
        if (sel.empty()) { say("복사할 걸 먼저 골라"); return; }
        int n = copyToClip(sel);
        char b[64]; std::snprintf(b, sizeof(b), "%d개 복사함 (Ctrl+V 로 붙여넣기)", n);
        say(b);
    };
    // Ctrl+V — 커서 자리에 꺼낸다
    auto doPaste = [&]() {
        if (clipboard.comps.empty()) { say("복사해 둔 게 없음"); return; }
        syncW();
        std::vector<int> made = pasteClip(wx, wy);
        sel = made;
        char b[64]; std::snprintf(b, sizeof(b), "%d개 붙여넣음", (int)made.size());
        say(b); touch();
    };
    // Ctrl+D — 그 자리에서 바로 복제 (담아 두지 않는다)
    auto doDup = [&]() {
        if (sel.empty()) { say("복제할 걸 먼저 골라"); return; }
        std::vector<int> made = duplicate(sel, 24, 24);
        char b[64]; std::snprintf(b, sizeof(b), "%d개 복제함", (int)made.size());
        say(b);
        sel = made; touch();
    };
    // Del — 고른 부품과 고른 선을 지운다
    auto doDelete = [&]() {
        int nc = 0, nw = 0;
        for (int i : sel)
            if (i >= 0 && i < (int)world.comps.size() && world.comps[i].alive) {
                deleteComp(world, i); ++nc;
            }
        if (selWire >= 0 && selWire < (int)world.wires.size() && world.wires[selWire].alive) {
            world.wires[selWire].alive = false; ++nw;
        }
        sel.clear(); selWire = -1;
        if (!nc && !nw) { say("지울 걸 먼저 골라"); return; }
        char b[64];
        if (nc && nw)      std::snprintf(b, sizeof(b), "부품 %d개·선 %d개 지움", nc, nw);
        else if (nc)       std::snprintf(b, sizeof(b), "부품 %d개 지움", nc);
        else               std::snprintf(b, sizeof(b), "선 %d개 지움", nw);
        say(b); touch();
    };
    // 핀 이름 바꾸기 (F2). 골라 둔 핀 하나, 없으면 커서 밑의 핀.
    auto doRenamePin = [&] {
        int target = -1, npin = 0;
        for (int i : sel)
            if (i >= 0 && i < (int)world.comps.size() && world.comps[i].alive
                && isPin(world.comps[i].type)) { target = i; ++npin; }
        if (npin > 1) { say("핀 하나만 골라야 이름을 고친다"); return; }
        if (target < 0) {
            syncW();
            int ci = compAt(wx, wy);
            if (ci >= 0 && isPin(world.comps[ci].type)) target = ci;
        }
        if (target < 0) { say("핀을 고르거나 핀 위에서 F2"); return; }
        startRename(target);
    };

    // 접기/펼치기 — 묶음·풀음의 상자를 작게 줄인다. 이어 둔 선은 그대로다.
    auto doFold = [&](int only) {
        std::vector<int> targets;
        if (only >= 0) targets.push_back(only);
        else targets = sel;
        int n = 0; bool nowFold = true;
        for (int i : targets) {
            if (i < 0 || i >= (int)world.comps.size()) continue;
            Comp& c = world.comps[i];
            if (!c.alive || !canFold(c)) continue;
            if (n == 0) nowFold = !isFolded(c);        // 첫 것에 맞춰 다 같이
            int cx0 = c.x + compW(c)/2, cy0 = c.y + compH(c)/2;
            c.aux = nowFold ? (c.aux | FOLD_BIT) : (c.aux & ~FOLD_BIT);
            c.x = cx0 - compW(c)/2; c.y = cy0 - compH(c)/2;   // 가운데는 그대로
            ++n;
        }
        if (!n) { say("접을 수 있는 건 묶음·풀음뿐 (3비트부터)"); return; }
        char b2[64];
        std::snprintf(b2, sizeof(b2), "%d개 %s", n, nowFold ? "접음" : "펼침");
        say(b2); touch();
    };

    // 화면 줄 수 바꾸기
    auto doRows = [&](int step) {
        int n = 0;
        for (int i : sel) {
            if (i < 0 || i >= (int)world.comps.size()) continue;
            Comp& c = world.comps[i];
            if (!c.alive || c.type != SCREEN) continue;
            int r = std::clamp(screenRows(c) + step, 1, SCR_MAX);
            c.aux = (c.aux & ~(0x1F << SCR_ROWS_SHIFT)) | (r << SCR_ROWS_SHIFT);
            resizePorts(c);
            for (auto& wr : world.wires)
                if (wr.alive && wr.to == i && wr.toPort >= nIn(c)) wr.alive = false;
            ++n;
        }
        if (!n) { say("줄 수는 화면만 바꾼다 (E)"); return; }
        char b2[48];
        std::snprintf(b2, sizeof(b2), "화면 %d줄", screenRows(world.comps[sel[0]]));
        say(b2); touch();
    };

    // 폭 바꾸기 — 핀·묶음·풀음만. 폭이 바뀌면 그 포트에 걸린 선은 안 맞으니 끊는다.
    auto doWidth = [&](int step) {
        int changed = 0, cut = 0;
        for (int i : sel) {
            if (i < 0 || i >= (int)world.comps.size()) continue;
            Comp& c = world.comps[i];
            if (!c.alive || !hasWidth(c.type) || c.chipId >= 0) continue;
            int w = std::clamp(widthOf(c) + step, 1, WIDTH_MAX);
            if (w == widthOf(c)) continue;
            c.aux = w;
            resizePorts(c);
            ++changed;
            for (auto& wr : world.wires) {
                if (!wr.alive) continue;
                if ((wr.from == i || wr.to == i) &&
                    !wireFits(world, wr.from, wr.fromPort, wr.to, wr.toPort)) {
                    wr.alive = false; ++cut;
                }
            }
        }
        if (!changed) { say("폭을 바꿀 수 있는 건 핀·묶음·풀음뿐"); return; }
        char b2[96];
        if (cut) std::snprintf(b2, sizeof(b2), "폭 바꿈 %d개 — 안 맞는 선 %d개 끊김", changed, cut);
        else     std::snprintf(b2, sizeof(b2), "폭 바꿈 %d개", changed);
        say(b2);
        touch();
    };
    // 클럭 빠르기
    auto doClockSpeed = [&](int step) {
        int changed = 0;
        for (int i : sel) {
            if (i < 0 || i >= (int)world.comps.size()) continue;
            Comp& c = world.comps[i];
            if (!c.alive || c.type != CLOCK) continue;
            c.aux = std::clamp(c.aux + step, 0, CLOCK_N - 1);
            ++changed;
        }
        if (changed) {
            char b2[64];
            std::snprintf(b2, sizeof(b2), "클럭 %d틱마다",
                          CLOCK_TICKS[std::clamp(world.comps[sel[0]].aux, 0, CLOCK_N - 1)]);
            say(b2); touch();
        }
    };

    auto doRotate = [&](int step) {
        if (sel.empty()) { say("돌릴 걸 먼저 골라 (R)"); return; }
        rotateSel(sel, step); touch();
    };

    // 그림 뽑기 — 장면 차리고, 몇 프레임 돌려 값이 자리잡은 뒤 저장한다.
    // 첫 화면 갈래도 여기를 지나야 해서 따로 뺐다 (안 그러면 그 화면에서 멈춘다).
    auto shotStep = [&] {
        if (shotAt < 0) return;
            if (shotWait > 0) {
                --shotWait;
            } else {
                static const char* NAMES[] = {
                    "셈틀-첫화면.ppm", "셈틀-단계목록.ppm", "셈틀-학습.ppm",
                    "셈틀-샌드박스.ppm", "셈틀-부품.ppm", "셈틀-고치기.ppm"
                };
                if (shotAt > 0) writePPM(ren, NAMES[shotAt - 1]);
                if (shotAt >= 6) { running = false; }
                else {
                    // 다음 장면 차리기
                    editStack.clear(); dropHandles(); hintOn = false; uiOn = true;
                    if (shotAt == 0) {                       // 1) 첫 화면
                        screen = SC_MENU;
                    } else if (shotAt == 1) {                // 2) 단계 목록
                        screen = SC_PICK; chips.clear();
                        lessonDone = lessonByTitle("여덟 칸 기억");
                        lessonAt = lessonDone;
                        pickScrollTo(lessonAt);
                    } else if (shotAt == 2) {                // 3) 학습 — XOR 단계 풀어 놓은 모습
                        screen = SC_LEARN; chips.clear();
                        lessonAt = lessonByTitle("XOR — 다를"); lessonDone = lessonAt;
                        setupLesson(lessonAt);
                        {
                            const Lesson& L = LESSONS[lessonAt];
                            int A = findPin(PIN_IN, L.inName[0]), B = findPin(PIN_IN, L.inName[1]);
                            int O = findPin(PIN_OUT, L.outName[0]);
                            int g = addComp(world, T_XOR, 300, 180);
                            addWire(world, A, 0, g, 0); addWire(world, B, 0, g, 1);
                            addWire(world, g, 0, O, 0);
                            setSwitch(A, 1);
                        }
                        hintOn = true;
                        fitView();
                    } else if (shotAt == 3) {                // 4) 샌드박스 — 진짜 저장본
                        screen = SC_SANDBOX;
                        if (!loadState()) { world = SubSim{}; seedDemo(); }
                        fitView();
                    } else if (shotAt == 4) {                // 5) 새 부품들
                        screen = SC_SANDBOX; world = SubSim{}; chips.clear();
                        int ck = addComp(world, CLOCK, 80, 260); world.comps[ck].aux = 3;
                        int bu = addComp(world, BUNDLE, 300, 120);
                        int sp = addComp(world, SPLIT, 700, 120);
                        int ram = addComp(world, RAM, 480, 300);
                        addWire(world, bu, 0, sp, 0);
                        world.comps[sp].aux |= FOLD_BIT;      // 한쪽은 접어 놓고 견줘 본다
                        // 스위치묶음과 화면도 같이 보여 준다
                        {
                            int sb2 = addComp(world, SWBANK, 120, 620);
                            world.comps[sb2].out.assign(1, 181);
                            int scr = addComp(world, SCREEN, 700, 640);
                            world.comps[scr].aux = 4 | (4 << SCR_ROWS_SHIFT);
                            resizePorts(world.comps[scr]);
                            static const Val PIC[4] = { 0x6, 0x9, 0x9, 0x6 };
                            for (int r = 0; r < 4; ++r) {
                                int q = addComp(world, SWBANK, 400, 560 + r * 150);
                                world.comps[q].aux = 4; resizePorts(world.comps[q]);
                                world.comps[q].out.assign(1, PIC[r]);
                                addWire(world, q, 0, scr, r);
                            }
                            addWire(world, sb2, 0, ram, 1);
                        }
                        for (int i = 0; i < 8; ++i) {
                            int q = addComp(world, SWITCH, 120, 20 + i * 46);
                            addWire(world, q, 0, bu, i);
                            if ((181 >> i) & 1) { auto& o = world.comps[q].out; if (!o.empty()) o[0] = 1; }
                        }
                        for (int i = 0; i < 8; ++i) {
                            int q = addComp(world, LAMP, 880, 20 + i * 46);
                            addWire(world, sp, i, q, 0);
                        }
                        addWire(world, ck, 0, ram, 3);
                        fitView();
                    } else {                                 // 6) 칩 고치는 중
                        screen = SC_SANDBOX;
                        if (!loadState()) { world = SubSim{}; seedDemo(); }
                        int target = -1;
                        for (int i = 0; i < (int)chips.size(); ++i)
                            if (chips[i].alive) { target = i; break; }
                        if (target >= 0) { beginEdit(target); fitView(); }
                    }
                    shotWait = 90;                           // 값이 자리잡을 시간
                }
                ++shotAt;
            }
    };

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            // ── 목록 찾기 글을 치는 중 ──
            if (findOn && !naming && !confirming) {
                if (e.type == SDL_TEXTINPUT) {
                    if (toolFind.size() < 24) { toolFind += e.text.text; toolScroll = 0; }
                    continue;
                }
                if (e.type == SDL_KEYDOWN) {
                    SDL_Keycode k = e.key.keysym.sym;
                    if (k == SDLK_BACKSPACE) {
                        if (!toolFind.empty()) {
                            toolFind.pop_back();
                            while (!toolFind.empty() && (toolFind.back() & 0xC0) == 0x80)
                                toolFind.pop_back();
                        }
                        toolScroll = 0;
                        continue;
                    }
                    if (k == SDLK_RETURN || k == SDLK_KP_ENTER) {
                        findOn = false; SDL_StopTextInput(); continue;
                    }
                    if (k == SDLK_ESCAPE) {
                        if (!toolFind.empty()) toolFind.clear();
                        else { findOn = false; SDL_StopTextInput(); }
                        toolScroll = 0;
                        continue;
                    }
                    continue;                       // 다른 키는 안 먹는다
                }
            }

            // ── 나갈까 묻는 중이면 그것만 받는다 ──
            if (confirming) {
                if (e.type == SDL_QUIT) running = false;
                else if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                    WIN_W = std::max(WIN_W_MIN, e.window.data1);
                    WIN_H = std::max(WIN_H_MIN, e.window.data2);
                } else if (e.type == SDL_MOUSEMOTION) { mx = e.motion.x; my = e.motion.y; }
                else if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                    if (confirmYes().has(e.button.x, e.button.y)) leaveToMenu();
                    else if (confirmNo().has(e.button.x, e.button.y)) confirming = false;
                } else if (e.type == SDL_KEYDOWN) {
                    SDL_Keycode k = e.key.keysym.sym;
                    if (k == SDLK_RETURN || k == SDLK_KP_ENTER || k == SDLK_y) leaveToMenu();
                    else if (k == SDLK_ESCAPE || k == SDLK_n) confirming = false;
                }
                continue;
            }

            // ── 이름 입력 중이면 그것만 받는다 ──
            if (naming) {
                if (e.type == SDL_TEXTINPUT) {
                    if (nameBuf.size() < 20) nameBuf += e.text.text;
                } else if (e.type == SDL_KEYDOWN) {
                    SDL_Keycode k = e.key.keysym.sym;
                    if (k == SDLK_BACKSPACE && !nameBuf.empty()) {
                        // UTF-8 한 글자(뒤쪽 연속 바이트까지) 지우기
                        nameBuf.pop_back();
                        while (!nameBuf.empty() && (nameBuf.back() & 0xC0) == 0x80) nameBuf.pop_back();
                    } else if (k == SDLK_RETURN || k == SDLK_KP_ENTER) {
                        if (renameComp >= 0) {                     // 핀 이름 고치기
                            if (renameComp < (int)world.comps.size() && !nameBuf.empty()) {
                                world.comps[renameComp].label = nameBuf; touch();
                            }
                        } else if (!nameBuf.empty()) {             // 칩 만들기
                            createChip(sel, nameBuf); sel.clear(); touch();
                        }
                        naming = false; renameComp = -1; SDL_StopTextInput();
                    } else if (k == SDLK_ESCAPE) {
                        naming = false; renameComp = -1; SDL_StopTextInput();
                    }
                } else if (e.type == SDL_QUIT) running = false;
                else if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                    WIN_W = std::max(WIN_W_MIN, e.window.data1);
                    WIN_H = std::max(WIN_H_MIN, e.window.data2);
                }
                continue;
            }

            // ── 단계 목록이면 고르는 것만 받는다 ──
            if (screen == SC_PICK) {
                if (e.type == SDL_QUIT) running = false;
                else if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                    WIN_W = std::max(WIN_W_MIN, e.window.data1);
                    WIN_H = std::max(WIN_H_MIN, e.window.data2);
                    pickScroll = std::clamp(pickScroll, 0, pickMaxScroll());
                }
                else if (e.type == SDL_MOUSEMOTION) { mx = e.motion.x; my = e.motion.y; }
                else if (e.type == SDL_MOUSEWHEEL) {
                    pickScroll = std::clamp(pickScroll - e.wheel.y * S(48), 0, pickMaxScroll());
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                    int at = pickHit(e.button.x, e.button.y);
                    if (at >= 0 && at <= lessonDone) {
                        screen = SC_LEARN;
                        if (at != lessonAt) gotoLesson(at);      // 지금 단계면 판을 안 건드린다
                    }
                } else if (e.type == SDL_KEYDOWN) {
                    SDL_Keycode k = e.key.keysym.sym;
                    if (k == SDLK_ESCAPE) leaveToMenu();
                    else if (k == SDLK_RETURN || k == SDLK_KP_ENTER) screen = SC_LEARN;
                    else if (k == SDLK_UP)   pickScroll = std::clamp(pickScroll - S(48), 0, pickMaxScroll());
                    else if (k == SDLK_DOWN) pickScroll = std::clamp(pickScroll + S(48), 0, pickMaxScroll());
                    else if (k == SDLK_LEFTBRACKET || k == SDLK_RIGHTBRACKET) {
                        uiScale += (k == SDLK_RIGHTBRACKET) ? 0.1f : -0.1f;
                        applyUiScale();
                        pickScroll = std::clamp(pickScroll, 0, pickMaxScroll());
                    }
                }
                continue;
            }

            // ── 첫 화면이면 고르는 것만 받는다 ──
            if (screen == SC_MENU) {
                if (e.type == SDL_QUIT) running = false;
                else if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                    WIN_W = std::max(WIN_W_MIN, e.window.data1);
                    WIN_H = std::max(WIN_H_MIN, e.window.data2);
                }
                else if (e.type == SDL_MOUSEMOTION) { mx = e.motion.x; my = e.motion.y; }
                else if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                    if (menuBtn(0).has(e.button.x, e.button.y)) openPick();
                    if (menuBtn(1).has(e.button.x, e.button.y)) enterMode(SC_SANDBOX);
                } else if (e.type == SDL_KEYDOWN) {
                    SDL_Keycode k = e.key.keysym.sym;
                    if (k == SDLK_1) openPick();
                    if (k == SDLK_2) enterMode(SC_SANDBOX);
                    if (k == SDLK_ESCAPE) running = false;
                    if (k == SDLK_LEFTBRACKET || k == SDLK_RIGHTBRACKET) {
                        uiScale += (k == SDLK_RIGHTBRACKET) ? 0.1f : -0.1f;
                        applyUiScale();
                        SDL_SetWindowMinimumSize(win, WIN_W_MIN, WIN_H_MIN);
                        if (WIN_W < WIN_W_MIN || WIN_H < WIN_H_MIN)
                            SDL_SetWindowSize(win, std::max(WIN_W, WIN_W_MIN),
                                                   std::max(WIN_H, WIN_H_MIN));
                    }
                }
                continue;
            }

            switch (e.type) {
                case SDL_QUIT: running = false; break;

                case SDL_WINDOWEVENT:
                    if (e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                        WIN_W = std::max(WIN_W_MIN, e.window.data1);
                        WIN_H = std::max(WIN_H_MIN, e.window.data2);
                    }
                    break;

                case SDL_KEYDOWN: {
                    SDL_Keycode k = e.key.keysym.sym;
                    bool ctrl = (SDL_GetModState() & KMOD_CTRL) != 0;
                    bool shift = (SDL_GetModState() & KMOD_SHIFT) != 0;
                    // Esc: 고른 게 있으면 그것만 놓고, 없으면 나갈지 묻는다
                    if (k == SDLK_ESCAPE) {
                        if (tool != 0 || !sel.empty() || selWire >= 0) {
                            tool = 0; sel.clear(); selWire = -1;
                        }
                        else if (hintOn) hintOn = false;
                        else if (!editStack.empty()) { doEndEdit(); break; }
                        else { confirming = true; break; }
                    }
                    else if (k == SDLK_p)          paused = !paused;
                    else if (k == SDLK_BACKQUOTE)  tool = 0;
                    else if (k == SDLK_g)          startNaming();
                    else if (k == SDLK_SLASH) { findOn = true; SDL_StartTextInput(); }
                    else if (k == SDLK_TAB) {
                        uiOn = !uiOn;
                        dropHandles();                 // 판이 사라지면 잡고 있던 것도 놓는다
                        if (!uiOn) say("Tab 으로 판 다시 보기");
                    }
                    else if (k == SDLK_LEFTBRACKET || k == SDLK_RIGHTBRACKET) {
                        uiScale += (k == SDLK_RIGHTBRACKET) ? 0.1f : -0.1f;
                        applyUiScale();
                        SDL_SetWindowMinimumSize(win, WIN_W_MIN, WIN_H_MIN);
                        if (WIN_W < WIN_W_MIN || WIN_H < WIN_H_MIN)
                            SDL_SetWindowSize(win, std::max(WIN_W, WIN_W_MIN),
                                                   std::max(WIN_H, WIN_H_MIN));
                        char b2[48]; std::snprintf(b2, sizeof(b2), "글자 크기 %d%%",
                                                   (int)(uiScale * 100 + 0.5f));
                        say(b2);
                        dirty = true; dirtyAt = frame;
                    }
                    else if (screen == SC_LEARN && k == SDLK_h) hintOn = !hintOn;
                    else if (screen == SC_LEARN && (k == SDLK_RETURN || k == SDLK_KP_ENTER)) doGrade();
                    else if (ctrl && k == SDLK_z)  { if (shift) doRedo(); else doUndo(); }
                    else if (ctrl && k == SDLK_y)  doRedo();
                    else if (ctrl && k == SDLK_s)  doExport();
                    else if (ctrl && k == SDLK_o)  doImport();
                    else if (k == SDLK_e)          doRows(shift ? -1 : 1);
                    else if (k == SDLK_F2)         doRenamePin();
                    else if (k == SDLK_q)          doFold(-1);
                    else if (k == SDLK_w)          doWidth(shift ? -1 : 1);
                    else if (k == SDLK_COMMA)      doClockSpeed(-1);
                    else if (k == SDLK_PERIOD)     doClockSpeed(1);
                    else if (k == SDLK_r)          doRotate(shift ? 3 : 1);
                    else if (ctrl && k == SDLK_c) doCopy();
                    else if (ctrl && k == SDLK_v) doPaste();
                    else if (ctrl && k == SDLK_d) doDup();
                    else if (k == SDLK_DELETE || k == SDLK_BACKSPACE) doDelete();
                    else if (k == SDLK_MINUS  || k == SDLK_KP_MINUS)
                        zoomAt(viewZoom / 1.25f, (canvasL() + canvasR())/2, WIN_H/2);
                    else if (k == SDLK_EQUALS || k == SDLK_KP_PLUS)
                        zoomAt(viewZoom * 1.25f, (canvasL() + canvasR())/2, WIN_H/2);
                    else if (k == SDLK_0)          { viewZoom = 1.0f; }
                    else if (k == SDLK_f)          fitView();
                    else if (k >= SDLK_1 && k <= SDLK_9) {
                        int t = k - SDLK_1 + 1;
                        if (t <= TYPE_N) tool = t;
                    }
                    break;
                }

                case SDL_MOUSEWHEEL: {
                    int cx, cy; SDL_GetMouseState(&cx, &cy);
                    if (uiOn && cx < PANEL_W) {              // 왼쪽 부품 목록 굴리기
                        toolScroll -= e.wheel.y * S(40);
                        clampToolScroll();
                        break;
                    }
                    if (cx < canvasL() || cx >= canvasR()) break;
                    if (SDL_GetModState() & KMOD_CTRL) {
                        zoomAt(viewZoom * (e.wheel.y > 0 ? 1.15f : 1/1.15f), cx, cy);
                    } else if (SDL_GetModState() & KMOD_SHIFT) {
                        viewX -= e.wheel.y * 60 / viewZoom;      // 가로 스크롤
                    } else {
                        viewY -= e.wheel.y * 60 / viewZoom;
                    }
                    break;
                }

                case SDL_MOUSEBUTTONDOWN: {
                    mx = e.button.x; my = e.button.y;
                    bool inPanel = uiOn && mx < PANEL_W;

                    // 학습 모드 오른쪽 설명판
                    if (uiOn && screen == SC_LEARN && mx >= canvasR()) {
                        if (e.button.button == SDL_BUTTON_LEFT) {
                            if (btnGrade().has(mx, my))   doGrade();
                            if (btnHint().has(mx, my))    hintOn = !hintOn;
                            if (btnPrevLes().has(mx, my)) gotoLesson(lessonAt - 1);
                            if (btnNextLes().has(mx, my)) gotoLesson(lessonAt + 1);
                            // 머리말을 누르면 단계 목록
                            int hts = lessonFit(LESSONS[std::clamp(lessonAt, 0, LESSON_N-1)], WIN_H).ts;
                            if (lesHdrLv(hts) && lesHdr(hts).has(mx, my)) openPick();
                        }
                        break;
                    }

                    if (e.button.button == SDL_BUTTON_LEFT) {
                        if (inPanel) {
                            bool handled = false;
                            // 찾기 칸을 누르면 글 치는 중으로
                            if (findBox().has(mx, my)) {
                                findOn = true; SDL_StartTextInput(); handled = true;
                            }
                            // 굴림 막대를 잡았나
                            if (!handled && toolScrollMax() > 0 && scrollTrack().has(mx, my)) {
                                dragScroll = true; handled = true;
                                Rect t = scrollTrack(), th = scrollThumb();
                                if (!th.has(mx, my)) {          // 막대 밖을 누르면 그리로 뛴다
                                    int rel = std::clamp(my - t.y - th.h / 2, 0, std::max(1, t.h - th.h));
                                    toolScroll = toolScrollMax() * rel / std::max(1, t.h - th.h);
                                    clampToolScroll();
                                }
                                scrollGrabY = my - scrollThumb().y;
                            }
                            std::vector<int> shown = shownTools();
                            int nt = (int)shown.size();
                            for (int row = 0; row < nt && !handled; ++row) {
                                if (!toolVisible(row)) continue;
                                int i = shown[row];
                                Rect r = toolRect(row);
                                int cid = toolChip(i);
                                // 지우기 단추가 줄 안에 들어 있으니 그쪽을 먼저 본다
                                if (cid >= 0 && toolEditBtn(row).has(mx, my)) {
                                    doEditChip(cid);
                                    handled = true;
                                } else if (cid >= 0 && toolDelBtn(row).has(mx, my)) {
                                    // 학습에서 상품으로 받은 칩은 못 지운다 — 뒷 단계가 막힌다
                                    bool prize = false;
                                    if (screen == SC_LEARN)
                                        for (int k = 0; k < LESSON_N && k < lessonDone; ++k)
                                            if (LESSONS[k].bank && LESSONS[k].name
                                                && chips[cid].name == LESSONS[k].name) prize = true;
                                    if (prize) {
                                        std::snprintf(toast, sizeof(toast),
                                                      "'%s' 는 단계에서 받은 부품이라 못 지움",
                                                      chips[cid].name.c_str());
                                        toastLeft = 200;
                                        handled = true;
                                        continue;
                                    }
                                    int uses = chipUses(cid);
                                    if (uses > 0) {
                                        std::snprintf(toast, sizeof(toast),
                                                      "'%s' 는 %d군데서 쓰는 중이라 못 지움",
                                                      chips[cid].name.c_str(), uses);
                                        toastLeft = 200;
                                    } else {
                                        std::snprintf(toast, sizeof(toast), "'%s' 지움",
                                                      chips[cid].name.c_str());
                                        toastLeft = 120;
                                        deleteChip(cid); touch();
                                        tool = 0;          // 뒤엣것 자리가 밀리니 손으로 되돌린다
                                    }
                                    handled = true;
                                } else if (r.has(mx, my)) {
                                    if (toolEnabled(i)) tool = (i == tool ? 0 : i);
                                    else say("이 단계에선 못 쓰는 부품");
                                    handled = true;
                                }
                            }
                            if (!handled && btnBundle().has(mx, my)) startNaming();
                            if (!handled && btnClear().has(mx, my)) {
                                world = SubSim{}; touch();
                                draggingWire = draggingComp = selecting = false;
                                wireFromComp = dragComp = -1; sel.clear();
                            }
                            break;
                        }
                        if (mx >= canvasR()) break;                   // 설명판 자리
                        syncW();
                        int oc, op;
                        if (outPortAt(wx, wy, oc, op)) {              // 출력 포트 잡기 → 선 끌기
                            draggingWire = true; wireFromComp = oc; wireFromPort = op; break;
                        }
                        if (findOn) { findOn = false; SDL_StopTextInput(); }
                        int onComp = compAt(wx, wy);
                        if (tool > 0) {
                            // 빈 자리를 누르면 놓고, 이미 있는 걸 누르면 잡아서 옮긴다
                            if (onComp < 0) { placeTool(tool, wx - GW_MIN/2, wy - 24); touch(); break; }
                            draggingComp = true; dragComp = onComp; dragMoved = false;
                            dragDX = wx - world.comps[onComp].x;
                            dragDY = wy - world.comps[onComp].y;
                            dragBase.clear();
                            break;
                        }
                        int ci = onComp;                             // 손: 부품 잡기 / 판: 상자선택
                        bool shiftNow = (SDL_GetModState() & KMOD_SHIFT) != 0;

                        // Shift 로 누르면 고른 목록에 넣거나 뺀다 (끌지는 않는다)
                        if (ci >= 0 && shiftNow) {
                            bool had = false;
                            for (size_t k = 0; k < sel.size(); ++k)
                                if (sel[k] == ci) { sel.erase(sel.begin() + k); had = true; break; }
                            if (!had) sel.push_back(ci);
                            selWire = -1;
                            char b2[48];
                            std::snprintf(b2, sizeof(b2), "%s · %d개 고름",
                                          had ? "뺌" : "넣음", (int)sel.size());
                            say(b2);
                            break;
                        }

                        // 칩 상자를 두 번 누르면 속으로 들어간다 (핀 이름은 F2)
                        if (ci >= 0 && e.button.clicks >= 2 && world.comps[ci].chipId >= 0) {
                            doEditChip(world.comps[ci].chipId); break;
                        }
                        // 묶음·풀음을 두 번 누르면 접거나 편다
                        if (ci >= 0 && e.button.clicks >= 2 && canFold(world.comps[ci])) {
                            doFold(ci); break;
                        }
                        if (ci >= 0) {
                            draggingComp = true; dragComp = ci; dragMoved = false;
                            dragDX = wx - world.comps[ci].x; dragDY = wy - world.comps[ci].y;
                            // 고른 것 안을 잡았으면 고른 것 전부가 같이 따라온다
                            dragBase.clear();
                            if (inSel(sel, ci))
                                for (int s : sel)
                                    if (s != ci && s >= 0 && s < (int)world.comps.size())
                                        dragBase.push_back({ s, world.comps[s].x - world.comps[ci].x,
                                                                world.comps[s].y - world.comps[ci].y });
                        } else {
                            int wi = wireAt(wx, wy);                 // 선을 눌렀나
                            if (wi >= 0) { selWire = wi; if (!shiftNow) sel.clear(); break; }
                            selWire = -1;
                            selecting = true; selAdd = shiftNow;
                            selX0 = selX1 = wx; selY0 = selY1 = wy;
                            if (!selAdd) sel.clear();                // Shift 면 있던 것에 더한다
                        }
                    } else if (e.button.button == SDL_BUTTON_MIDDLE) {
                        if (inPanel) break;                          // 화면 끌기
                        panning = true; panSX = mx; panSY = my; panVX = viewX; panVY = viewY;
                    } else if (e.button.button == SDL_BUTTON_RIGHT) {
                        // 선 위에서는 바로 끊고, 그 밖에서는 손 도구로 돌아간다
                        if (!inPanel) {
                            syncW();
                            int wi = wireAt(wx, wy);
                            if (wi >= 0) {
                                world.wires[wi].alive = false;
                                if (selWire == wi) selWire = -1;
                                touch();
                                break;
                            }
                        }
                        if (tool != 0) { tool = 0; say("손 도구"); }
                        else if (!sel.empty() || selWire >= 0) { sel.clear(); selWire = -1; }
                    }
                    break;
                }

                case SDL_MOUSEMOTION: {
                    mx = e.motion.x; my = e.motion.y;
                    syncW();
                    hoverTool = -1;
                    if (uiOn && mx < PANEL_W) {
                        std::vector<int> shown = shownTools();
                        for (int row = 0; row < (int)shown.size(); ++row)
                            if (toolVisible(row) && toolChip(shown[row]) >= 0 &&
                                (toolDelBtn(row).has(mx, my) || toolEditBtn(row).has(mx, my))) {
                                hoverTool = row; break;
                            }
                    }
                    if (dragScroll) {
                        Rect t = scrollTrack(), th = scrollThumb();
                        int span = std::max(1, t.h - th.h);
                        toolScroll = toolScrollMax() * std::clamp(my - scrollGrabY - t.y, 0, span) / span;
                        clampToolScroll();
                    }
                    if (panning) {
                        viewX = panVX - (mx - panSX) / viewZoom;
                        viewY = panVY - (my - panSY) / viewZoom;
                    }
                    // 포트 위에 있으면 이름을 띄울 준비
                    tipComp = -1;
                    if (mx >= canvasL() && mx < canvasR() && !draggingComp && !selecting) {
                        int c2, p2;
                        if (outPortAt(wx, wy, c2, p2))      { tipComp = c2; tipPort = p2; tipIsIn = false; }
                        else if (inPortAt(wx, wy, c2, p2))  { tipComp = c2; tipPort = p2; tipIsIn = true; }
                    }
                    if (draggingComp && dragComp >= 0) {
                        int nx = wx - dragDX, ny = wy - dragDY;
                        if (nx != world.comps[dragComp].x || ny != world.comps[dragComp].y) dragMoved = true;
                        world.comps[dragComp].x = nx; world.comps[dragComp].y = ny;
                        for (const auto& f : dragBase)               // 같이 고른 것들도 따라온다
                            if (f.i < (int)world.comps.size()) {
                                world.comps[f.i].x = nx + f.dx;
                                world.comps[f.i].y = ny + f.dy;
                            }
                    }
                    if (selecting) { selX1 = wx; selY1 = wy; }
                    break;
                }

                case SDL_MOUSEBUTTONUP: {
                    mx = e.button.x; my = e.button.y; syncW();
                    if (e.button.button == SDL_BUTTON_MIDDLE) panning = false;
                    if (e.button.button == SDL_BUTTON_LEFT) dragScroll = false;
                    if (e.button.button == SDL_BUTTON_LEFT) {
                        if (draggingWire) {
                            int comp, port;
                            if (inPortAt(wx, wy, comp, port)) {
                                addWire(world, wireFromComp, wireFromPort, comp, port); touch();
                            }
                            draggingWire = false; wireFromComp = -1;
                        }
                        if (draggingComp && dragComp >= 0 && dragComp < (int)world.comps.size()) {
                            // 입력핀도 눌러서 켠다. 안 그러면 묶기 전에 회로를 못 돌려 본다.
                            int t = world.comps[dragComp].type;
                            bool flipped = false;
                            if (!dragMoved && (t == SWITCH || t == PIN_IN)
                                && !isBitBank(world.comps[dragComp])) {
                                auto& o = world.comps[dragComp].out;
                                if (!o.empty()) { o[0] = !o[0]; flipped = true; }
                            }
                            // 여러 비트짜리는 누른 칸의 비트만 뒤집는다
                            if (!dragMoved && isBitBank(world.comps[dragComp])) {
                                Comp& c = world.comps[dragComp];
                                (void)t;
                                int bit = swBankBitAt(c, wx, wy);
                                if (bit >= 0 && !c.out.empty()) {
                                    c.out[0] = maskTo((Val)(c.out[0] ^ (1u << bit)), widthOf(c));
                                    flipped = true;
                                }
                            }
                            if (dragMoved) touch();        // 옮긴 건 되돌릴 수 있어야 한다
                            else if (flipped) touchRun();  // 껐다 켠 건 저장만
                            draggingComp = false; dragComp = -1; dragBase.clear();
                        }
                        if (selecting) {
                            selecting = false;
                            int rx0 = std::min(selX0, selX1), ry0 = std::min(selY0, selY1);
                            int rx1 = std::max(selX0, selX1), ry1 = std::max(selY0, selY1);
                            if (!selAdd) sel.clear();
                            for (int i = 0; i < (int)world.comps.size(); ++i) {
                                if (!world.comps[i].alive) continue;
                                Comp& c = world.comps[i];
                                int w = compW(c), h = compH(c);
                                if (c.x < rx1 && rx0 < c.x + w && c.y < ry1 && ry0 < c.y + h) {
                                    if (!inSel(sel, i)) sel.push_back(i);   // 두 번 안 들어가게
                                }
                            }
                            selAdd = false;
                        }
                    }
                    break;
                }
            }
        }

        if (!paused) tickSub(world);

        // 바뀐 뒤 잠깐 잠잠하면 저장한다. 끄는 손에도 안 놓치게 나갈 때 한 번 더.
        // 그림 뽑는 중에는 저장하지 않는다 — 진짜 저장본을 장면용 판으로 덮으면 안 된다
        if (dirty && !smoke && shotAt < 0 && ansAt < 0 && editStack.empty() && frame - dirtyAt > 45) {
            if (saveState()) { dirty = false; savedFlash = 90; }
            else             { dirty = false; say("저장 실패"); }
        }

        if (smoke) {
            // 배율을 한계까지 훑고, 도중에 돌리고 복사하고, 세 화면을 다 밟는다
            viewZoom = ZOOM_MIN + (ZOOM_MAX - ZOOM_MIN) * (smokeLeft % 12) / 11.0f;
            viewX += 37; viewY -= 21;
            if (smokeLeft == 26) { sel = { 0, 1, 2 }; rotateSel(sel, 1); }
            if (smokeLeft == 22) duplicate({ 0, 1, 2 }, 200, 200);
            if (smokeLeft == 20) fitView();
            // 창 크기를 최소~큼직까지 흔들어 본다
            { int k = smokeLeft % 6;
              WIN_W = (k < 3) ? WIN_W_MIN : 1600;
              WIN_H = (k % 2) ? WIN_H_MIN : 900; }
            if (smokeLeft == 27 && !chips.empty()) beginEdit(0);   // 칩 고치는 화면
            if (smokeLeft == 25 && !editStack.empty()) endEdit();
            if (smokeLeft == 24) uiOn = false;                  // 판 숨긴 채로
            if (smokeLeft == 20) uiOn = true;
            if (smokeLeft == 18) screen = SC_MENU;               // 첫 화면
            if (smokeLeft == 16) {                               // 단계 목록
                screen = SC_PICK; lessonDone = 12; lessonAt = 12; pickScrollTo(lessonAt);
            }
            if (smokeLeft == 14) { screen = SC_LEARN; setupLesson(0); }
            if (smokeLeft == 10) doGrade();                      // 못 푼 채로 채점
            if (smokeLeft == 6)  { lessonDone = LESSON_N; lessonAt = LESSON_N - 1;
                                   setupLesson(lessonAt); hintOn = true; }   // 마지막 단계 + 힌트
            if (smokeLeft == 4)  uiOn = false;                   // 학습 모드에서 판 숨기기
            if (smokeLeft == 2)  { uiOn = true; }
            if (--smokeLeft <= 0) running = false;
        }

        // ── 그리기 ──
        if (screen == SC_PICK) {
            drawPick(ren, mx, my);
            // 목록이 통째로 비는 일이 없게 — 지금 단계 표시가 실제로 찍혔나 본다
            if (smoke) {
                int ry = pickTop() + pickRowY(lessonAt) - pickScroll;
                if (ry >= pickTop() && ry + pickLesH() <= pickBot() &&
                    !hasColor(ren, pickColX(), ry, pickColW(), pickLesH(), 0x5A86BA)) {
                    std::fprintf(stderr, "단계 목록에 지금 단계가 안 보인다\n");
                    smokeBad = true;
                }
            }
            shotStep();
            SDL_RenderPresent(ren);
            ++frame;
            continue;
        }
        if (screen == SC_MENU) {
            int hov = -1;
            for (int i = 0; i < 2; ++i) if (menuBtn(i).has(mx, my)) hov = i;
            drawMenu(ren, hov);
            shotStep();
            SDL_RenderPresent(ren);
            ++frame;
            continue;
        }

        setCol(ren, COL_BG); SDL_RenderClear(ren);
        // 판은 캔버스 안에만 그린다 (학습 모드는 오른쪽에 설명판이 있다)
        SDL_Rect clip{ canvasL(), 0, canvasR() - canvasL(), WIN_H };
        SDL_RenderSetClipRect(ren, &clip);
        drawGrid(ren);

        drawnSelWire = selWire;
        for (const auto& w : world.wires)
            if (w.alive && world.comps[w.from].alive && world.comps[w.to].alive) drawWire(ren, w);

        if (draggingWire && wireFromComp >= 0) {
            const Comp& a = world.comps[wireFromComp];
            int x0, y0, d0x, d0y;
            outPort(a, wireFromPort, x0, y0); outDir(a, d0x, d0y);
            int pts[24][2];
            bezierD(x0, y0, d0x, d0y, wx, wy, -d0x, -d0y, pts, 24);
            for (int k = 0; k < 23; ++k)
                thickLine(ren, w2sX((float)pts[k][0]),   w2sY((float)pts[k][1]),
                               w2sX((float)pts[k+1][0]), w2sY((float)pts[k+1][1]),
                          std::max(1, w2sLen(3)), 0x8890A0);
        }

        for (int i = 0; i < (int)world.comps.size(); ++i)
            if (world.comps[i].alive) drawComp(ren, i, sel);

        if (selecting) {
            int rx0 = std::min(selX0, selX1), ry0 = std::min(selY0, selY1);
            Rect r = toScreen(rx0, ry0, std::abs(selX1-selX0), std::abs(selY1-selY0));
            fillRect(ren, r, COL_SEL, 40);
            frameRect(ren, r, COL_SEL);
        }

        SDL_RenderSetClipRect(ren, nullptr);

        // 고치는 중이면 위에 띠를 두른다 — 판이 아니라 설계도를 보고 있다는 표시
        if (!editStack.empty()) {
            std::string path;
            for (size_t i = 0; i < editStack.size(); ++i) {
                if (i) path += " > ";
                path += chips[editStack[i].chipId].name;
            }
            path = "고치는 중:  " + path;
            int bh = S(30);
            fillRect(ren, { canvasL(), 0, canvasR() - canvasL(), bh }, 0x2C4A6A, 235);
            fillRect(ren, { canvasL(), bh, canvasR() - canvasL(), S(2) }, 0x5A86BA);
            drawText(ren, canvasL() + S(14), (bh - S(14)) / 2, 14, 0xD8E8FF, path.c_str());
            const char* out = "Esc : 나가서 반영";
            drawText(ren, canvasR() - textWidth(12, out) - S(14), (bh - S(12)) / 2, 12,
                     0x9FC0FF, out);
        }

        if (uiOn) {
            drawPanel(ren, tool, (int)sel.size(), hoverTool, frame);
            if (screen == SC_LEARN) drawLessonPanel(ren, lesMsg, lesOK, lesLeft);
        } else {
            // 판이 없으면 어떻게 되돌리는지만 구석에 남겨 둔다
            drawText(ren, S(12), WIN_H - S(26), 11, 0x50565E, "Tab : 판 다시 보기");
            if (screen == SC_LEARN) {
                char pr[64];
                std::snprintf(pr, sizeof(pr), "%d / %d 단계 · Enter 채점 · H 힌트",
                              lessonAt + 1, LESSON_N);
                drawText(ren, S(12), S(12), 12, 0x707684, pr);
            }
        }
        if (lesLeft > 0) --lesLeft;
        if (paused) drawText(ren, canvasL() + S(12), editStack.empty() ? S(12) : S(44),
                             14, 0xE0C060, "멈춤 (P)");

        {   // 오른위에 배율과 조작 요약
            int right = canvasR() - S(12);
            // 고치는 중이면 위에 띠가 있으니 그만큼 내려서 안 겹치게
            int top = editStack.empty() ? S(10) : S(42);
            char z[32]; std::snprintf(z, sizeof(z), "%d%%", (int)(viewZoom * 100 + 0.5f));
            drawText(ren, right - textWidth(13, z), top, 13, COL_DIM, z);
            if (savedFlash > 0) {                       // 저장됐다고 잠깐 표시
                --savedFlash;
                const char* s = "저장됨";
                drawText(ren, right - textWidth(11, s), top + S(38), 11,
                         savedFlash > 30 ? 0x6C9E7A : 0x3A4A40, s);
            }
            const char* hint = !sel.empty()
                ? "Shift+클릭=하나씩 · R=돌리기 · Q=접기 · F2=핀 이름 · Ctrl+C/V · G=묶기"
                : (selWire >= 0
                   ? "선 고름 — Del 로 지운다"
                   : (tool != 0
                      ? "빈 곳=놓기 · 부품 잡고 끌면 옮기기 · 우클릭=손 도구 (선 위면 끊기)"
                      : (screen == SC_LEARN
                         ? "Enter=채점 · 끌어서 고르기(Shift 로 더하기) · 선은 우클릭으로 끊기"
                         : "끌어서 고르기(Shift 로 더하기) · Del=지우기 · 선은 우클릭으로 끊기")));
            drawText(ren, right - textWidth(11, hint), top + S(20), 11, 0x60646E, hint);
        }

        if (toastLeft > 0) {
            --toastLeft;
            int tw = textWidth(14, toast);
            Rect tb{ (canvasL() + canvasR() - tw) / 2 - S(14), WIN_H - S(62), tw + S(28), S(36) };
            uint8_t a = (uint8_t)std::min(230, toastLeft * 8);
            fillRect(ren, tb, 0x2A2028, a);
            frameRect(ren, tb, 0x7A5A62, a);
            drawText(ren, tb.x + S(14), tb.y + (tb.h - S(14))/2, 14, 0xF0D0D8, toast);
        }

        // 포트 이름 말풍선
        if (tipComp >= 0 && tipComp < (int)world.comps.size() && world.comps[tipComp].alive
            && !draggingWire && !naming) {
            const Comp& c = world.comps[tipComp];
            const char* pn = portName(c, tipPort, tipIsIn);
            char nm[64];
            if (pn) std::snprintf(nm, sizeof(nm), "%s", pn);
            else if (c.chipId >= 0)
                std::snprintf(nm, sizeof(nm), "%s %d", tipIsIn ? "입력" : "출력", tipPort + 1);
            else if (isPin(c.type) || nIn(c) + nOut(c) > 0)
                std::snprintf(nm, sizeof(nm), "%s", compName(c));
            else nm[0] = 0;

            // 몇 비트짜리인지, 지금 값이 얼마인지 같이 보여 준다
            char b[128] = "";
            if (nm[0]) {
                int wd = tipIsIn ? inWidth(c, tipPort) : outWidth(c, tipPort);
                Val v  = tipIsIn ? (tipPort < (int)c.in.size()  ? c.in[tipPort]  : 0)
                                 : (tipPort < (int)c.out.size() ? c.out[tipPort] : 0);
                if (wd > 1) std::snprintf(b, sizeof(b), "%s · %d비트 · %d", nm, wd, (int)v);
                else        std::snprintf(b, sizeof(b), "%s · %d", nm, (int)(v ? 1 : 0));
            }
            if (b[0]) {
                int px, py;
                if (tipIsIn) inPort(c, tipPort, px, py); else outPort(c, tipPort, px, py);
                drawTip(ren, w2sX((float)px), w2sY((float)py), b);
            }
        }

        if (screen == SC_LEARN && hintOn) drawHint(ren);

        if (confirming) {
            int hv = -1;
            if (confirmYes().has(mx, my)) hv = 0;
            if (confirmNo().has(mx, my))  hv = 1;
            drawConfirm(ren, hv);
        }

        if (naming)
            drawNaming(ren, nameBuf, frame,
                       renameComp >= 0 ? "핀 이름:" : "커스텀 게이트 이름:",
                       renameComp >= 0 ? "Enter 로 고치기 · Esc 로 취소"
                                       : "Enter 로 만들기 · Esc 로 취소");

        // 정답 그림 — 단계마다 정답을 짓고 값이 자리잡은 뒤 한 장
        if (ansAt >= 0) {
            if (ansWait > 0) {
                --ansWait;
            } else {
                if (ansAt > 0) {                     // 앞 단계 그림을 저장하고 칩으로 담는다
                    const Lesson& done = LESSONS[ansAt - 1];
                    // 제목에서 앞의 "12. " 를 떼고 쓴다 (파일 이름에 번호가 두 번 들어가지 않게)
                    const char* t2 = done.title;
                    while (*t2 && (*t2 == ' ' || (*t2 >= '0' && *t2 <= '9') || *t2 == '.')) ++t2;
                    char path[192];
                    std::snprintf(path, sizeof(path), "정답/%02d-%s.ppm", ansAt, *t2 ? t2 : done.name);
                    writePPM(ren, path);
                    bankLesson(done);
                }
                if (ansAt >= LESSON_N) { running = false; }
                else {
                    screen = SC_LEARN; uiOn = true; hintOn = false;
                    lessonAt = ansAt; lessonDone = ansAt;
                    setupLesson(ansAt);
                    char err[200] = "";
                    if (!buildSolution(ansAt, err, sizeof(err)))
                        std::fprintf(stderr, "%d단계 정답을 못 지음: %s\n", ansAt + 1, err);
                    fitView();
                    // 값이 자리잡을 때까지는 그리지 말고 시뮬만 돌린다 (그리면서 기다리면 아주 느리다)
                    for (int k = 0; k < 200; ++k) tickSub(world);
                    ansWait = 2;
                }
                ++ansAt;
            }
        }

        shotStep();

        // 판이 통째로 안 그려지는 일이 있었다(자르기를 안 풀어서). 스모크에서 잡는다.
        if (smoke && uiOn && screen != SC_MENU) {
            if (!anyPaint(ren, 0, 0, PANEL_W, WIN_H, COL_BG)) {
                std::fprintf(stderr, "왼쪽 도구판이 안 그려졌다\n");
                smokeBad = true;
            }
            if (screen == SC_LEARN &&
                !anyPaint(ren, WIN_W - LES_W, 0, LES_W, WIN_H, COL_BG)) {
                std::fprintf(stderr, "오른쪽 설명판이 안 그려졌다\n");
                smokeBad = true;
            }
        }

        SDL_RenderPresent(ren);
        ++frame;
    }

    if (shotAt >= 0 || ansAt >= 0) { editStack.clear(); dirty = false; }   // 그림 모드는 안 쓴다
    while (!editStack.empty()) endEdit();       // 고치던 칩 마무리
    if (dirty && !smoke) saveState();          // 나가는 길에 못 쓴 것 마저

    for (auto& kv : glyphCache) if (kv.second.tex) SDL_DestroyTexture(kv.second.tex);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return smokeBad ? 1 : 0;
}
