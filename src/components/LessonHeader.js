import React from 'react';
import { useLocation } from '@docusaurus/router';

export default function LessonHeader({ title, subtitle, chapter, lessonNumber, progress }) {
  const location = useLocation();

  return (
    <div className="bg-gradient-to-r from-blue-50 to-teal-50 border-b border-gray-200 p-6 mb-8 rounded-lg shadow-sm">
      <div className="flex flex-col md:flex-row md:items-center md:justify-between gap-4">
        <div>
          <div className="flex items-center gap-2 text-sm text-gray-600 mb-1">
            <span className="bg-blue-100 text-blue-800 px-2 py-1 rounded-full text-xs font-medium">
              Chapter {chapter}
            </span>
            <span>Lesson {lessonNumber}</span>
          </div>
          <h1 className="text-3xl font-bold text-gray-900 mb-2">{title}</h1>
          {subtitle && <p className="text-lg text-gray-600">{subtitle}</p>}
        </div>

        {progress && (
          <div className="flex flex-col items-end">
            <div className="w-32 bg-gray-200 rounded-full h-2 mb-1">
              <div
                className="bg-blue-600 h-2 rounded-full transition-all duration-300 ease-out"
                style={{ width: `${progress}%` }}
              ></div>
            </div>
            <span className="text-sm text-gray-600">{progress}% complete</span>
          </div>
        )}
      </div>
    </div>
  );
}