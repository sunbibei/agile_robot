#include <sstream>
#include <string>
#include <vector>
#include <stack>
#include <map>
#include <exception>
#include <cmath>
using namespace std;

#include "foundation/internal/expr.h"

namespace internal {

bool s_is_init = false;

template<class T>
class Stack : public vector<T>
{
public:
    void push(const T& val){vector<T>::push_back(val);}
    T&   top() {
      /*if(vector<T>::empty())
        {throw ExprError("syntax error");}*/
      return vector<T>::back();
    }
    void pop() {
      /*if(vector<T>::empty())
        {throw ExprError("syntax error");}*/
      vector<T>::pop_back();}
};


class OPD
{
public:
    double* ref;
    double  num;
    OPD():ref(0),num(0){}
    OPD(double* r):ref(r),num(*r){}
    OPD(double  n):ref(0),num(n){}
    operator double() const {
        if (ref) {return *ref;}
        else {return num;}
    }
    operator double* const() {
        if (!ref) return nullptr;
        return ref;
    }
};

struct OprInfo
{
    typedef void (*OprFunction)(Stack<OPD> &);
    const char* symbol;    // + - * / ...
    OprFunction eval;
    int         level;
    int         opdInc;    //operand count increment
    int         rAss;       //Right-associative

};

#define OPERATOR2(name, opr, level)     \
void _##name(Stack<OPD> &sNum)       \
{                                       \
    double r = sNum.top(); sNum.pop();  \
    double l = sNum.top(); sNum.pop();  \
    sNum.push(l opr r);                 \
}

#define OPERATOR2r(name, opr, level)     \
void _##name(Stack<OPD> &sNum)       \
{                                       \
    double r = sNum.top(); sNum.pop();  \
    double &l = *sNum.top(); sNum.pop();  \
    sNum.push( &(l opr r) );                 \
}

#define OPERATOR2i(name, opr, level)     \
void _##name(Stack<OPD> &sNum)       \
{                                       \
    double r = sNum.top(); sNum.pop();  \
    double l = sNum.top(); sNum.pop();  \
    sNum.push( int(l) opr int(r) );                 \
}

#define OPERATOR2ir(name, opr, level)     \
void _##name(Stack<OPD> &sNum)       \
{                                       \
    double r = sNum.top(); sNum.pop();  \
    double &l = *sNum.top(); sNum.pop();  \
    int li = l;                             \
    l = li opr int(r);                          \
    sNum.push( &l );                        \
}

#define OPERATOR1(name, opr, level)     \
void _##name(Stack<OPD> &sNum)       \
{                                       \
    double r = sNum.top(); sNum.pop();  \
    sNum.push( opr r );                 \
}

#define OPERATOR1r(name, opr, level)     \
void _##name(Stack<OPD> &sNum)       \
{                                       \
    double &r = *sNum.top(); sNum.pop();  \
    sNum.push( &(opr r) );                 \
}

#define OPERATOR1b(name, opr, level)     \
void _##name(Stack<OPD> &sNum)       \
{                                       \
    double &r = *sNum.top(); sNum.pop();  \
    double t = r opr;                              \
    sNum.push( t );                 \
}

#define OPERATOR1i(name, opr, level)     \
void _##name(Stack<OPD> &sNum)       \
{                                       \
    double r = sNum.top(); sNum.pop();  \
    sNum.push( opr int(r) );                 \
}

#define OPFUNCTION1(name)     \
void _##name(Stack<OPD> &sNum)       \
{                                       \
    double r = sNum.top(); sNum.pop();  \
    sNum.push( name(r) );                 \
}

// define operators function
void _placeHolder(Stack<OPD> &sNum) {return;}
void _pow(Stack<OPD> &sNum) {
    double r = sNum.top(); sNum.pop();
    double l = sNum.top(); sNum.pop();
    sNum.push( pow(l,r) );
}
void _xor(Stack<OPD> &sNum) {
    double r = sNum.top(); sNum.pop();
    double l = sNum.top(); sNum.pop();
    sNum.push( int(l)^int(r) );
}

#define OPR_DEC \
OPERATOR1(plus, + , 2)  \
OPERATOR1(minus, - , 2) \
OPERATOR2(add, +, 5)    \
OPERATOR2(sub, -, 5)    \
OPERATOR2(mul, *, 4)    \
OPERATOR2(div, /, 4)    \
OPERATOR2i(mod, %, 4)   \
OPERATOR1r(fpp, ++ , 2) \
OPERATOR1r(fmm, -- , 2) \
OPERATOR1b(bpp, ++ , 1) \
OPERATOR1b(bmm, -- , 1) \
OPERATOR2(and, &&, 12)  \
OPERATOR2(or , ||, 13)  \
OPERATOR1i(not, ! , 2)  \
OPERATOR2i(andb, &, 9)  \
OPERATOR2i(orb , |, 11) \
OPERATOR1i(notb, ~, 2)  \
OPERATOR2i(lsh, <<, 6)  \
OPERATOR2i(rsh, >>, 6)  \
OPERATOR2r(ass , =, 15) \
OPERATOR2r(ada , +=, 15) \
OPERATOR2r(sua , -=, 15) \
OPERATOR2r(mua , *=, 15) \
OPERATOR2r(dia , /=, 15) \
OPERATOR2ir(moa , %=, 15) \
OPERATOR2ir(ana , &=, 15) \
OPERATOR2ir(ora , |=, 15) \
OPERATOR2ir(lsa , <<=, 15) \
OPERATOR2ir(rsa , >>=, 15) \
OPERATOR2(gr , >, 7) \
OPERATOR2(le , <, 7) \
OPERATOR2(gre, >=,7) \
OPERATOR2(lee, <=,7) \
OPERATOR2(equ, ==,8) \
OPERATOR2(nqu, !=,8) \
OPFUNCTION1(sin) \
OPFUNCTION1(cos) \
OPFUNCTION1(tan) \
OPFUNCTION1(asin) \
OPFUNCTION1(acos) \
OPFUNCTION1(atan) \
OPFUNCTION1(sqrt) \
OPFUNCTION1(exp) \
OPFUNCTION1(abs) \
OPFUNCTION1(ceil) \
OPFUNCTION1(floor) \
OPFUNCTION1(log) \
OPFUNCTION1(int) \
OPFUNCTION1(bool)

OPR_DEC

//defein operators table
#undef  OPERATOR2
#undef  OPERATOR2r
#undef  OPERATOR2i
#undef  OPERATOR2ir
#undef  OPERATOR1i
#undef  OPERATOR1
#undef  OPERATOR1r
#undef  OPERATOR1b
#undef  OPFUNCTION1

#define OPERATOR2(name, opr, level)     {#opr,_##name,level,-1,0},
#define OPERATOR2r(name, opr, level)     OPERATOR2(name, opr, level)

#define OPERATOR2i(name, opr, level)    OPERATOR2(name, opr, level)
#define OPERATOR2ir(name, opr, level)    OPERATOR2(name, opr, level)
#define OPERATOR1(name, opr, level)    {#opr,_##name,level, 0,1},
#define OPERATOR1r(name, opr, level)    OPERATOR1(name, opr, level)
#define OPERATOR1b(name, opr, level)    {#opr,_##name,level, 0,0},
#define OPERATOR1i(name, opr, level)    OPERATOR1(name, opr, level)
#define OPFUNCTION1(name)                {#name,_##name,2, 0,1},

typedef const OprInfo* OPR;
OprInfo oprTable[] =
{
    {";",_placeHolder,0xFE,0,1},
    {"(",_placeHolder,0xFD,0,0},
    {")",_placeHolder,0xFC,0,0},

    {"^",_pow,3,-1,0},
    {"><",_xor,10,-1,0},
    {"xor",_xor,10,-1,0},
    OPR_DEC
    {",",_placeHolder,16,0,0},
};

map<string,OPR> oprs;
char    cType[256] = {0};   // char types bitmap

typedef unsigned char int8;
#define CT_WSP  1
#define CT_NUM  2
#define CT_OPR  4
#define CT_SYM  8

//inline bool isHex(char c){ return (c>='0' && c<='9')||c=='.'; }
inline bool isNum(char c){ return cType[int8(c)]&CT_NUM; }
inline bool isOp(char c){  return cType[int8(c)]&CT_OPR; }
inline bool isWS(char c){  return cType[int8(c)]&CT_WSP; }// 08~0d,30
inline bool isSym(char c){ return cType[int8(c)]&CT_SYM; }
inline bool isOp(string t){return oprs.find(t)!=oprs.end(); }


int ExprInit()
{
    for (size_t i=0;i<sizeof(oprTable)/sizeof(*oprTable);i++)
    {
        oprs[ oprTable[i].symbol ] = oprTable+i;
    }

    for (int c=0;c<256;c++)
    {
        if ( (c>='0' && c<='9')||c=='.' ) {cType[c] |= CT_NUM;}
        if ( (c>='!'&&c<='/') || (c>=':'&&c<='@')|| (c>='['&&c<='^')||(c>='{'&&c<='~') ) {cType[c] |= CT_OPR;}
        if ( (c>='A' && c<='Z')||(c>='a' && c<='z')||(c>='0' && c<='9')||(c=='_')||(c=='[')||(c==']') ) {cType[c] |= CT_SYM;}
        if ( c==' '||c=='\t'||c=='\r'||c=='\n' ) {cType[c] |= CT_WSP;}
    }
    s_is_init = true;
    return 0;
}

const OPR OEND =  oprTable+0; // ;
const OPR LPAR = oprTable+1; // (
const OPR RPAR = oprTable+2; // )
const OPR OPLUS = oprTable+3; // +1
const OPR OMINUS = oprTable+4; // -1


enum TokenType
{
    T_NUM = 0,
    T_OPR = 1,
    T_SYM = 2,
    //T_OPD = 4;
};

struct Token
{
    TokenType type; // 0:opr  1:opd
//    union
//    {
        OPR     opr;
        OPD     opd;
//    };
};

const Token TEND={T_OPR, OEND};

double stringToNum(const string& str)
{
    double ret = 0;
    stringstream ss;
    ss.str(str);
    ss>>ret;
    return ret;
}




typedef string::iterator SPtr;
void skipWhiteSpace(const string& str, SPtr& it)
{
    while( it != str.end() && isWS(*it) ){it++;}
}

double skipNumber(const string& str, SPtr& it)
{
    SPtr begin = it;
    while( it != str.end() && isNum(*it) ){it++;}
    return stringToNum(string(begin, it) );
}

OPR skipOperator(const string& str, SPtr& it)
{
    SPtr begin = it++;
    string sym(begin, it);
    map<string,OPR>::iterator existOp=oprs.find(sym);
    map<string,OPR>::iterator nextOp=oprs.end();
    if (existOp == nextOp){return OEND;}
    while ( it!=str.end() )
    {
        it++;
        sym = string(begin, it);
        nextOp = oprs.find(sym);
        if (nextOp != oprs.end())
        {
            existOp = nextOp;
        } else {
            it--;
            break;
        }
    }
    return existOp->second;
}

string skipSymbol(const string& str, SPtr& it)
{
    SPtr begin = it;
    while( it != str.end() && isSym(*it) ){it++;}
    return string(begin, it);
}



bool tokenize(string str, map<string,double>& vars, vector<Token>& ret)
{
    int numCount = 0;
    int parCount = 0;

    Token token;
    for (SPtr ptr = str.begin();ptr!=str.end();)
    {
        if ((numCount & ~1) || (parCount < 0)) return false;

        if (isWS(*ptr) ) {skipWhiteSpace(str, ptr); }
        else if (isNum(*ptr) ){
            token.type = T_NUM;
            token.opd = OPD(skipNumber(str, ptr));
            ret.push_back(token);
            numCount ++;
        }
        else if (isOp(*ptr) ) {
            token.type = T_OPR;
            token.opr = skipOperator(str, ptr);

            if ( (token.opr->eval == _bpp||token.opr->eval == _bmm
                ||token.opr->eval == _add||token.opr->eval == _sub)
                && numCount==0)
            {token.opr -= 2;}

            ret.push_back(token);
            numCount += token.opr->opdInc;

            if (LPAR == token.opr) {parCount++;}
            if (RPAR == token.opr) {parCount--;}

            if (token.opr == OEND)  // check balance
            {
              if ((numCount & ~1) || (parCount < 0)) return false;
                numCount = 0; //reset
            }
        }
        else if (isSym(*ptr) ){
            string sym = skipSymbol(str, ptr);
            if (isOp(sym) )
            {
                token.type = T_OPR;
                token.opr = oprs[sym];
                ret.push_back(token);
            } else if (vars.find(sym)!=vars.end()){
                token.type = T_NUM;
                token.opd = OPD( &(vars[sym]) );
                ret.push_back(token);
                numCount++;
            } else {
                string err = "unrecongized symbol: ";
                err += sym;
                return false;
            }
        }
        else {
            string err = "unrecongized symbol: ";
            err += *ptr;
            return false;
            ptr++;
        }
    }

    ret.push_back(TEND);

    if ((numCount & ~1) || (parCount != 0)) return false;
    return true;
}

bool eval(const string& exp, double& out)
{
  if (!s_is_init) ExprInit();

    Stack<OPD>  sOpd;
    Stack<OPR>  sOpr;

    sOpr.push(OEND);

    map<string,double> vars;
    vector<Token> tokens;
    if (!tokenize(exp, vars, tokens)) return false;

    for (vector<Token>::iterator it=tokens.begin();it!=tokens.end();it++ )
    {
        if (it->type == T_NUM)
        {
            sOpd.push(it->opd);
        }
        else if (it->type == T_OPR)
        {
            if (LPAR == it->opr) {sOpr.push(it->opr);}
            else
            {
                OPR prevOp = sOpr.top();
                while ( (prevOp->level < it->opr->level) ||
                        ((prevOp->level == it->opr->level)&&!it->opr->rAss)
                        )
                {
                    sOpr.pop();
                    prevOp->eval(sOpd);
                    prevOp = sOpr.top();
                }

                if (RPAR == it->opr) {
                    if (prevOp != LPAR) return false;
                    sOpr.pop();
                } else {
                    sOpr.push(it->opr);
                }
            }
        }
    }

    if ( sOpd.size()<1 ) return false;

    out = sOpd.top();
    return true;
}

} /* namespace internal */


