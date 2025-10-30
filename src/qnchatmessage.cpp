/*-------------------------------------------------
#
# Project created by QtCreator
# Author: 沙振宇
# CreateTime: 2018-07-23
# UpdateTime: 2019-12-27
# Info: Qt5气泡式聊天框——QListWidget+QPainter实现
# Url:https://shazhenyu.blog.csdn.net/article/details/81505832
# Github:https://github.com/ShaShiDiZhuanLan/Demo_MessageChat_Qt
#
#-------------------------------------------------*/
#include "qnchatmessage.h"
#include "KokoroTTS.h"
#include <QFontMetrics>
#include <QPaintEvent>
#include <QDateTime>
#include <QPainter>
#include <QMovie>
#include <QLabel>
#include <QDebug>
#include <QPushButton>
QNChatMessage::QNChatMessage(QWidget *parent) : QWidget(parent)
{
    QFont te_font = this->font();
    te_font.setFamily("MicrosoftYaHei");
    te_font.setPointSize(12);
    te_font.setWordSpacing(0);
    te_font.setLetterSpacing(QFont::PercentageSpacing,0);
    te_font.setLetterSpacing(QFont::PercentageSpacing, 100);          //300%,100为默认  //设置字间距%
    te_font.setLetterSpacing(QFont::AbsoluteSpacing, 0);             //设置字间距为3像素 //设置字间距像素值
    this->setFont(te_font);
    m_leftPixmap = QPixmap(":/resources/icons/robot.png");
    m_rightPixmap = QPixmap(":/resources/icons/User.png");

    // 新增：初始化语音按钮
    SpeakBtn = new QPushButton(this);
    SpeakBtn->setFixedSize(24, 24);  // 设置按钮大小
    SpeakBtn->setStyleSheet(R"(
        QPushButton {
            border: none;
            background: transparent;
            image: url(:/resources/icons/sound_icon.png);  /* 替换为你的语音图标路径 */
        }
        QPushButton:hover {
            image: url(:/resources/icons/sound_hover.png);  /* 可选：悬停图标 */
        }
    )");
    SpeakBtn->hide();  // 初始隐藏，根据气泡类型显示
    connect(SpeakBtn, &QPushButton::clicked, this, &QNChatMessage::on_SpeakBtn_clicked);
}

void QNChatMessage::setText(QString text, QString time, QSize allSize, QNChatMessage::User_Type userType)
{
    m_msg = text;
    m_userType = userType;
    m_time = time;
    m_curTime = QDateTime::fromTime_t(time.toInt()).toString("hh:mm");
    m_allSize = allSize;

    // 新增/修改：调整语音按钮位置
    if (userType == User_Me || userType == User_She) {  // 仅消息气泡显示按钮
        if (userType == User_Me) {  // 自己的气泡（右侧），按钮放左侧
            SpeakBtn->move(
                m_kuangRightRect.x() - SpeakBtn->width() - 10,  // 气泡左侧（按钮宽度+边距10）
                m_kuangRightRect.y() + m_kuangRightRect.height()/2 - SpeakBtn->height()/2  // 垂直居中
            );
        } else {  // 对方的气泡（左侧），按钮放右侧
            SpeakBtn->move(
                m_kuangLeftRect.x() + m_kuangLeftRect.width() + 10,  // 气泡右侧（边距10）
                m_kuangLeftRect.y() + m_kuangLeftRect.height()/2 - SpeakBtn->height()/2  // 垂直居中
            );
        }
        SpeakBtn->show();
    } else {
        SpeakBtn->hide();  // 时间气泡不显示按钮
    }

    this->update();
}

QSize QNChatMessage::fontRect(QString str)
{
    m_msg = str;
    int minHei = 30;
    int iconWH = 40;
    int iconSpaceW = 20;
    int iconRectW = 5;
    int iconTMPH = 10;
    int sanJiaoW = 6;
    int kuangTMP = 20;
    int textSpaceRect = 12;
    // m_kuangWidth = this->width() - kuangTMP - 2*(iconWH+iconSpaceW+iconRectW);
    m_kuangWidth = this->width() - kuangTMP - (iconWH+iconSpaceW+iconRectW);
    // m_textWidth = m_kuangWidth - 2*textSpaceRect + 10;
    m_textWidth = m_kuangWidth - 20;
    m_spaceWid = this->width() - m_textWidth;
    m_iconLeftRect = QRect(iconSpaceW, iconTMPH, iconWH, iconWH);
    m_iconRightRect = QRect(this->width() - iconSpaceW - iconWH, iconTMPH, iconWH, iconWH);

    QSize size = getRealString(m_msg); // 整个的size

    qDebug() << "fontRect Size:" << size;
    int hei = size.height() < minHei ? minHei : size.height();

    m_sanjiaoLeftRect = QRect(iconWH+iconSpaceW+iconRectW, m_lineHeight/2, sanJiaoW, hei - m_lineHeight);
    m_sanjiaoRightRect = QRect(this->width() - iconRectW - iconWH - iconSpaceW - sanJiaoW, m_lineHeight/2, sanJiaoW, hei - m_lineHeight);

    if(size.width() < (m_textWidth+m_spaceWid)) {
        m_kuangLeftRect.setRect(m_sanjiaoLeftRect.x()+m_sanjiaoLeftRect.width(), m_lineHeight/4*3, size.width()-m_spaceWid+2*textSpaceRect, hei-m_lineHeight);
        m_kuangRightRect.setRect(this->width() - size.width() + m_spaceWid - 2*textSpaceRect - iconWH - iconSpaceW - iconRectW - sanJiaoW,
                                 m_lineHeight/4*3, size.width()-m_spaceWid+2*textSpaceRect, hei-m_lineHeight);
    } else {
        m_kuangLeftRect.setRect(m_sanjiaoLeftRect.x()+m_sanjiaoLeftRect.width(), m_lineHeight/4*3, m_kuangWidth, hei-m_lineHeight);
        m_kuangRightRect.setRect(iconWH + kuangTMP + iconSpaceW + iconRectW - sanJiaoW, m_lineHeight/4*3, m_kuangWidth, hei-m_lineHeight);
    }
    m_textLeftRect.setRect(m_kuangLeftRect.x()+textSpaceRect,m_kuangLeftRect.y()+iconTMPH,
                           m_kuangLeftRect.width()-2*textSpaceRect,m_kuangLeftRect.height()-2*iconTMPH);
    m_textRightRect.setRect(m_kuangRightRect.x()+textSpaceRect,m_kuangRightRect.y()+iconTMPH,
                            m_kuangRightRect.width()-2*textSpaceRect,m_kuangRightRect.height()-2*iconTMPH);

    return QSize(size.width(), hei);
}

// QSize QNChatMessage::getRealString(QString src)
// {
//     QFontMetricsF fm(this->font());
//     m_lineHeight = fm.lineSpacing();
//     int nCount = src.count("\n");
//     int nMaxWidth = 0;
//     if(nCount == 0) {
//         nMaxWidth = fm.width(src);
//         QString value = src;
//         if(nMaxWidth > m_textWidth) {
//             nMaxWidth = m_textWidth;
//             int size = m_textWidth / fm.width(" ");
//             int num = fm.width(value) / m_textWidth;
//             int ttmp = num*fm.width(" ");
//             num = ( fm.width(value) ) / m_textWidth;
//             nCount += num;
//             QString temp = "";
//             for(int i = 0; i < num; i++) {
//                 temp += value.mid(i*size, (i+1)*size) + "\n";
//             }
//             src.replace(value, temp);
//         }
//     } else {
//         for(int i = 0; i < (nCount + 1); i++) {
//             QString value = src.split("\n").at(i);
//             nMaxWidth = fm.width(value) > nMaxWidth ? fm.width(value) : nMaxWidth;
//             if(fm.width(value) > m_textWidth) {
//                 nMaxWidth = m_textWidth;
//                 int size = m_textWidth / fm.width(" ");
//                 int num = fm.width(value) / m_textWidth;
//                 num = ((i+num)*fm.width(" ") + fm.width(value)) / m_textWidth;
//                 nCount += num;
//                 QString temp = "";
//                 for(int i = 0; i < num; i++) {
//                     temp += value.mid(i*size, (i+1)*size) + "\n";
//                 }
//                 src.replace(value, temp);
//             }
//         }
//     }
//     return QSize(nMaxWidth+m_spaceWid, (nCount + 1) * m_lineHeight+2*m_lineHeight);
// }

QSize QNChatMessage::getRealString(QString src)
{
    QFontMetricsF fm(this->font());
    m_lineHeight = fm.lineSpacing();
    
    // 设置文本选项
    QTextOption option;
    option.setWrapMode(QTextOption::WrapAtWordBoundaryOrAnywhere);
    option.setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    
    // 计算文本边界
    QRectF bounds;
    bounds = fm.boundingRect(QRectF(0, 0, m_textWidth, INT_MAX), 
                           option.alignment() | Qt::TextWordWrap, 
                           src);
    
    // 确保返回的宽度不会导致换行
    int width = qMin((int)bounds.width() + m_spaceWid, m_textWidth)+ fm.width(src.right(1));
    // if(width == m_textWidth) {
    //     width += fm.width(src.right(5));  // 为最后一个字符预留空间
    // }
    // width += fm.width(src.right(5));
    return QSize(width, (int)bounds.height() + 2 * m_lineHeight);
}

void QNChatMessage::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);//消锯齿
    painter.setPen(Qt::NoPen);
    painter.setBrush(QBrush(Qt::gray));

    if(m_userType == User_Type::User_She) { // 用户
        //头像
        // painter.drawRoundedRect(m_iconLeftRect,m_iconLeftRect.width(),m_iconLeftRect.height());
        painter.drawPixmap(m_iconLeftRect, m_leftPixmap);

        //框加边
        QColor col_KuangB(234, 234, 234);
        painter.setBrush(QBrush(col_KuangB));
        painter.drawRoundedRect(m_kuangLeftRect.x()-1,m_kuangLeftRect.y()-1,m_kuangLeftRect.width()+2,m_kuangLeftRect.height()+2,4,4);
        //框
        QColor col_Kuang(255,255,255);
        painter.setBrush(QBrush(col_Kuang));
        painter.drawRoundedRect(m_kuangLeftRect,4,4);

        //三角
        QPointF points[3] = {
            QPointF(m_sanjiaoLeftRect.x(), 30),
            QPointF(m_sanjiaoLeftRect.x()+m_sanjiaoLeftRect.width(), 25),
            QPointF(m_sanjiaoLeftRect.x()+m_sanjiaoLeftRect.width(), 35),
        };
        QPen pen;
        pen.setColor(col_Kuang);
        painter.setPen(pen);
        painter.drawPolygon(points, 3);

        //三角加边
        QPen penSanJiaoBian;
        penSanJiaoBian.setColor(col_KuangB);
        painter.setPen(penSanJiaoBian);
        painter.drawLine(QPointF(m_sanjiaoLeftRect.x() - 1, 30), QPointF(m_sanjiaoLeftRect.x()+m_sanjiaoLeftRect.width(), 24));
        painter.drawLine(QPointF(m_sanjiaoLeftRect.x() - 1, 30), QPointF(m_sanjiaoLeftRect.x()+m_sanjiaoLeftRect.width(), 36));

        //内容
        QPen penText;
        penText.setColor(QColor(51,51,51));
        painter.setPen(penText);

        QTextOption option(Qt::AlignLeft | Qt::AlignVCenter);
        option.setWrapMode(QTextOption::WrapAtWordBoundaryOrAnywhere);
        option.setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

        painter.setFont(this->font());
        painter.drawText(m_textLeftRect, m_msg,option);
    }  
    else if(m_userType == User_Type::User_Me) 
    { // 自己
        //头像
        //        painter.drawRoundedRect(m_iconRightRect,m_iconRightRect.width(),m_iconRightRect.height());
        painter.drawPixmap(m_iconRightRect, m_rightPixmap);

        //框
        QColor col_Kuang(75,164,242);
        painter.setBrush(QBrush(col_Kuang));
        painter.drawRoundedRect(m_kuangRightRect,4,4);

        //三角
        QPointF points[3] = {
            QPointF(m_sanjiaoRightRect.x()+m_sanjiaoRightRect.width(), 30),
            QPointF(m_sanjiaoRightRect.x(), 25),
            QPointF(m_sanjiaoRightRect.x(), 35),
        };
        QPen pen;
        pen.setColor(col_Kuang);
        painter.setPen(pen);
        painter.drawPolygon(points, 3);

        //内容
        QPen penText;
        penText.setColor(Qt::white);
        painter.setPen(penText);
        QTextOption option(Qt::AlignLeft | Qt::AlignVCenter);
        option.setWrapMode(QTextOption::WrapAtWordBoundaryOrAnywhere);
        painter.setFont(this->font());
        painter.drawText(m_textRightRect,m_msg,option);
    }  
    else if(m_userType == User_Type::User_Time) 
    { // 时间
        QPen penText;
        penText.setColor(QColor(153,153,153));
        painter.setPen(penText);
        QTextOption option(Qt::AlignCenter);
        option.setWrapMode(QTextOption::WrapAtWordBoundaryOrAnywhere);
        QFont te_font = this->font();
        te_font.setFamily("MicrosoftYaHei");
        te_font.setPointSize(10);
        painter.setFont(te_font);
        painter.drawText(this->rect(),m_curTime,option);
    }
}
void QNChatMessage::on_SpeakBtn_clicked() {
    qDebug() << "触发语音朗读，文本内容：" << m_msg;
    
    // 通过信号触发TTS工作线程进行处理
    emit TTSNotifier::instance()->textSpeaking(m_msg);
    
}
