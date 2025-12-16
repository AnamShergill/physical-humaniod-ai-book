import React from 'react';

export default function CalloutBlock({ type = 'info', title, children, icon }) {
  const styles = {
    info: {
      bg: 'bg-blue-50',
      border: 'border-blue-200',
      text: 'text-blue-800',
      icon: 'ℹ️'
    },
    warning: {
      bg: 'bg-amber-50',
      border: 'border-amber-200',
      text: 'text-amber-800',
      icon: '⚠️'
    },
    success: {
      bg: 'bg-green-50',
      border: 'border-green-200',
      text: 'text-green-800',
      icon: '✅'
    },
    note: {
      bg: 'bg-purple-50',
      border: 'border-purple-200',
      text: 'text-purple-800',
      icon: '📝'
    }
  };

  const style = styles[type];

  return (
    <div className={`border-l-4 ${style.border} ${style.bg} p-4 my-6 rounded-r-lg transition-all duration-200 hover:shadow-md`}>
      <div className="flex items-start gap-3">
        <span className="text-xl">{icon || style.icon}</span>
        <div className="flex-1">
          {title && <h4 className={`font-semibold ${style.text} mb-2`}>{title}</h4>}
          <div className={style.text}>{children}</div>
        </div>
      </div>
    </div>
  );
}